#!/usr/bin/env python3
"""
MJPEG video server with artificial latency injection.
View at http://<IP>:8080/video in Quest browser.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler
from socketserver import ThreadingTCPServer
from threading import Thread, Lock
import time
from collections import deque
import argparse

# Optional depth-mode dependencies (Phase 2b). The app runs fine without them;
# the "use depth" checkbox just falls back to flat-plane (Phase 2a).
try:
    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation
    DEPTH_AVAILABLE = True
except ImportError:
    DEPTH_AVAILABLE = False

# Configurable artificial latency (milliseconds)
ARTIFICIAL_LATENCY_MS = 0

# Lazy-loaded Depth Anything V2 model (only loaded when depth mode is first enabled)
_DEPTH_MODEL = None
_DEPTH_PROCESSOR = None
_DEPTH_DEVICE = None
_DEPTH_LOCK = Lock()  # protects loader from race when multiple clients first hit depth-mode


def _ensure_depth_model():
    """Lazy-load the depth model on first use. Raises RuntimeError if unavailable."""
    global _DEPTH_MODEL, _DEPTH_PROCESSOR, _DEPTH_DEVICE
    if not DEPTH_AVAILABLE:
        raise RuntimeError("Depth mode requires: pip install torch transformers pillow")
    with _DEPTH_LOCK:
        if _DEPTH_MODEL is None:
            _DEPTH_DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
            if _DEPTH_DEVICE == 'cpu':
                print("WARNING: depth mode running on CPU will be very slow (~2s/frame)")
            print(f"Loading Depth Anything V2 Small on {_DEPTH_DEVICE} (first use)...")
            model_id = "depth-anything/Depth-Anything-V2-Small-hf"
            _DEPTH_PROCESSOR = AutoImageProcessor.from_pretrained(model_id)
            _DEPTH_MODEL = AutoModelForDepthEstimation.from_pretrained(model_id).to(_DEPTH_DEVICE).eval()
            print("Depth model ready.")
    return _DEPTH_MODEL, _DEPTH_PROCESSOR, _DEPTH_DEVICE


def estimate_depth_tensor(frame_bgr, scene_depth_m):
    """Run Depth Anything V2 on a BGR frame; return metric depth as a torch tensor on GPU.

    Depth Anything outputs *relative* inverse depth; we calibrate to metric by
    forcing the median pixel's depth to equal scene_depth_m. The caller's
    scene_depth_m thus doubles as a global scale knob.

    Returns: torch.Tensor of shape (H, W), float32, on _DEPTH_DEVICE, depth in meters.
    """
    model, processor, device = _ensure_depth_model()
    h, w = frame_bgr.shape[:2]
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    inputs = processor(images=frame_rgb, return_tensors="pt").to(device)
    with torch.no_grad():
        predicted = model(**inputs).predicted_depth  # (1, H', W'), inverse-depth-ish
    # Upsample to original resolution
    predicted = torch.nn.functional.interpolate(
        predicted.unsqueeze(1), size=(h, w), mode='bicubic', align_corners=False
    ).squeeze(1).squeeze(0)  # (H, W)
    # Normalize to [0, 1] then invert into a depth-like quantity
    dmin, dmax = predicted.min(), predicted.max()
    norm = (predicted - dmin) / (dmax - dmin + 1e-6)  # 0=far, 1=near
    eps = 0.05
    raw_depth = 1.0 / (norm + eps)  # higher = farther
    # Calibrate: force median to equal user-specified scene_depth_m
    scale = scene_depth_m / raw_depth.median()
    depth_m = (raw_depth * scale).clamp(0.1, 100.0)
    return depth_m


def predict_view_depth(frame_bgr, depth_m_tensor, dx_robot, dy_robot, dyaw_rad, hfov_rad):
    """Per-pixel depth-aware reprojection. Forward-splat with GPU z-buffer.

    Voids appear where (a) no source pixel maps to that destination (disocclusion
    or off-frame), or (b) the source pixel is behind the new camera (Z <= 0).
    """
    if not DEPTH_AVAILABLE:
        raise RuntimeError("predict_view_depth requires torch")
    device = depth_m_tensor.device
    h, w = frame_bgr.shape[:2]
    fx = (w / 2.0) / np.tan(hfov_rad / 2.0)
    cx, cy = w / 2.0, h / 2.0

    # Transform parameters in old-camera frame
    t_x = -float(dy_robot)
    t_z = float(dx_robot)
    c = float(np.cos(dyaw_rad))
    s = float(np.sin(dyaw_rad))

    frame_t = torch.from_numpy(np.ascontiguousarray(frame_bgr)).to(device)  # HxWx3 uint8
    depth_t = depth_m_tensor.float()  # HxW

    # Pixel grid
    u_grid, v_grid = torch.meshgrid(
        torch.arange(w, dtype=torch.float32, device=device),
        torch.arange(h, dtype=torch.float32, device=device),
        indexing='xy',
    )

    # Back-project to old-camera 3D points
    X_old = (u_grid - cx) * depth_t / fx
    Y_old = (v_grid - cy) * depth_t / fx
    Z_old = depth_t

    # Translate then rotate: P_new = R_y(+dyaw) @ (P_old - t)
    X_sh = X_old - t_x
    Z_sh = Z_old - t_z
    X_new = c * X_sh + s * Z_sh
    Y_new = Y_old
    Z_new = -s * X_sh + c * Z_sh

    valid_z = Z_new > 1e-3
    Z_safe = torch.where(valid_z, Z_new, torch.ones_like(Z_new))
    u_new = fx * X_new / Z_safe + cx
    v_new = fx * Y_new / Z_safe + cy
    u_int = u_new.long()
    v_int = v_new.long()
    valid = valid_z & (u_int >= 0) & (u_int < w) & (v_int >= 0) & (v_int < h)

    HW = h * w
    # Use a dummy slot at the end for invalid pixels so we never index out of bounds
    flat_idx = (v_int * w + u_int).clamp(0, HW - 1)
    flat_idx = torch.where(valid, flat_idx, torch.full_like(flat_idx, HW - 1)).flatten()
    Z_flat = torch.where(valid, Z_new, torch.full_like(Z_new, float('inf'))).flatten()

    # Z-buffer: for each destination pixel, keep the min source Z
    zbuf = torch.full((HW,), float('inf'), device=device)
    zbuf.scatter_reduce_(0, flat_idx, Z_flat, reduce='amin', include_self=True)

    # Write the color of the winning source pixel into each destination
    is_min = (zbuf.gather(0, flat_idx) == Z_flat) & valid.flatten()
    src_flat = frame_t.reshape(-1, 3)
    output_flat = torch.zeros((HW, 3), dtype=torch.uint8, device=device)
    output_flat[flat_idx[is_min]] = src_flat[is_min.nonzero(as_tuple=True)[0]]

    return output_flat.reshape(h, w, 3).cpu().numpy()

class VideoBuffer:
    """Thread-safe frame buffer with latency injection"""
    def __init__(self):
        self.lock = Lock()
        self.frame_queue = deque()  # (timestamp, frame) pairs
        self.latency_ms = ARTIFICIAL_LATENCY_MS
        self.predict_enabled = False
        self.use_depth = False  # Phase 2b: per-pixel depth reprojection
        self.hfov_rad = 1.085  # ~62 deg, typical TurtleBot3 Gazebo camera
        self.scene_depth_m = 2.0  # plane depth (flat) / median depth (depth-mode)

    def set_latency(self, ms):
        with self.lock:
            self.latency_ms = ms
            print(f"Latency set to {ms}ms, buffer has {len(self.frame_queue)} frames")

    def set_predict(self, enabled):
        with self.lock:
            self.predict_enabled = bool(enabled)
            print(f"Prediction {'ON' if self.predict_enabled else 'OFF'}")

    def set_use_depth(self, enabled):
        with self.lock:
            want = bool(enabled)
            if want and not DEPTH_AVAILABLE:
                print("Depth mode requested but torch/transformers/pillow are not installed; "
                      "falling back to flat-plane. Install with: pip install torch transformers pillow")
                self.use_depth = False
                return
            self.use_depth = want
            print(f"Depth mode {'ON' if self.use_depth else 'OFF'}")

    def set_hfov_deg(self, deg):
        with self.lock:
            self.hfov_rad = float(deg) * np.pi / 180.0
            print(f"HFOV set to {deg} deg ({self.hfov_rad:.3f} rad)")

    def set_scene_depth_m(self, m):
        with self.lock:
            self.scene_depth_m = max(0.1, float(m))  # clip to >0 to avoid singular H
            print(f"Scene depth set to {self.scene_depth_m:.2f} m")

    def add_frame(self, frame):
        with self.lock:
            now = time.time()
            self.frame_queue.append((now, frame))
            # Keep 12 seconds of frames (to support 10s latency)
            cutoff = now - 12.0
            while self.frame_queue and self.frame_queue[0][0] < cutoff:
                self.frame_queue.popleft()

    def get_frame(self):
        with self.lock:
            if not self.frame_queue:
                return None

            now = time.time()
            delay_sec = self.latency_ms / 1000.0
            target_time = now - delay_sec

            # If latency is 0, return newest frame
            if self.latency_ms == 0:
                return self.frame_queue[-1][1]

            # Find the frame that's at least delay_sec old
            # Return the newest frame that is older than target_time
            best_frame = None
            for ts, frame in self.frame_queue:
                if ts <= target_time:
                    best_frame = frame
                else:
                    break

            # If no frame is old enough yet, return oldest available
            # (this happens when latency is first set high)
            if best_frame is None and self.frame_queue:
                best_frame = self.frame_queue[0][1]
                best_ts = self.frame_queue[0][0]
            else:
                best_ts = now  # fallback

            return best_frame

    def get_frame_with_age(self):
        """Returns (frame, age_in_ms, timestamp). timestamp is None if no frame."""
        with self.lock:
            if not self.frame_queue:
                return None, 0, None

            now = time.time()
            delay_sec = self.latency_ms / 1000.0
            target_time = now - delay_sec

            # If latency is 0, return newest frame
            if self.latency_ms == 0:
                ts, frame = self.frame_queue[-1]
                return frame, (now - ts) * 1000, ts

            # Find the frame that's at least delay_sec old
            best_frame = None
            best_ts = now
            for ts, frame in self.frame_queue:
                if ts <= target_time:
                    best_frame = frame
                    best_ts = ts
                else:
                    break

            # If no frame is old enough yet, return oldest available
            if best_frame is None and self.frame_queue:
                best_ts, best_frame = self.frame_queue[0]

            age_ms = (now - best_ts) * 1000 if best_frame is not None else 0
            return best_frame, age_ms, (best_ts if best_frame is not None else None)


class TwistBuffer:
    """Thread-safe ring buffer of (timestamp, lin, ang) cmd_vel samples.
    Used to integrate the robot's commanded motion over a given window."""
    def __init__(self):
        self.lock = Lock()
        self.queue = deque()  # (timestamp, lin, ang) triples

    def add(self, lin, ang):
        with self.lock:
            now = time.time()
            self.queue.append((now, lin, ang))
            cutoff = now - 12.0
            while self.queue and self.queue[0][0] < cutoff:
                self.queue.popleft()

    def integrate(self, t_from, t_to):
        """Returns (dx, dy, dyaw) in the robot's body frame at t_from, integrated
        from t_from to t_to. Zero-order hold: each sample's velocity is held until
        the next sample (or t_to for the last). Velocity before the earliest
        sample is assumed zero."""
        with self.lock:
            if not self.queue or t_from >= t_to:
                return 0.0, 0.0, 0.0
            samples = list(self.queue)

        dx, dy, dyaw = 0.0, 0.0, 0.0
        for i, (t_i, lin_i, ang_i) in enumerate(samples):
            t_next = samples[i + 1][0] if i + 1 < len(samples) else t_to
            a = max(t_i, t_from)
            b = min(t_next, t_to)
            if b <= a:
                continue
            dt = b - a
            # Closed-form integration for constant (lin, ang) over dt, starting
            # at heading dyaw. Yields exact circular-arc / straight-line motion.
            if abs(ang_i) > 1e-6:
                r = lin_i / ang_i
                new_dyaw = dyaw + ang_i * dt
                dx += r * (np.sin(new_dyaw) - np.sin(dyaw))
                dy += r * (-np.cos(new_dyaw) + np.cos(dyaw))
                dyaw = new_dyaw
            else:
                dx += lin_i * np.cos(dyaw) * dt
                dy += lin_i * np.sin(dyaw) * dt
        return dx, dy, dyaw


def predict_view(frame, dx_robot, dy_robot, dyaw_rad,
                 hfov_rad=1.085, scene_depth_m=2.0):
    """Warp a delayed frame by a predicted robot motion. Voids appear black.

    Inputs are in the robot's body frame at frame-capture time (REP-103):
      dx_robot:  forward displacement, m (+ = forward)
      dy_robot:  lateral displacement, m (+ = left)
      dyaw_rad:  yaw rotation,        rad (+ = CCW from above / left turn)

    Uses a plane-induced homography H = K (R - t n^T / d) K^-1 under the flat-
    plane proxy: the whole scene is assumed to sit at scene_depth_m. Yaw is
    depth-independent (any plane works); forward motion produces an expansion
    about the principal point; lateral motion produces a horizontal shift.
    True parallax (objects at different depths moving by different amounts)
    requires per-pixel depth -- that is Phase 2b.

    hfov_rad: camera horizontal FOV (default 1.085 ~ 62 deg, TurtleBot3 Gazebo).
    scene_depth_m: assumed scene distance for the flat-plane proxy.
    """
    if abs(dx_robot) < 1e-3 and abs(dy_robot) < 1e-3 and abs(dyaw_rad) < 1e-6:
        return frame

    # Clip forward motion to keep H non-singular when approaching the plane
    d = max(scene_depth_m, 0.1)
    dx_robot = float(np.clip(dx_robot, -10.0 * d, 0.8 * d))

    h, w = frame.shape[:2]
    fx = (w / 2.0) / np.tan(hfov_rad / 2.0)
    K = np.array([[fx, 0.0, w / 2.0],
                  [0.0, fx, h / 2.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    K_inv = np.linalg.inv(K)

    # Robot body -> camera frame (OpenCV camera: +X right, +Y down, +Z forward):
    #   +x_body (forward) -> +z_cam
    #   +y_body (left)    -> -x_cam
    t = np.array([-dy_robot, 0.0, dx_robot], dtype=np.float64)

    # Yaw: robot CCW around body +z is camera around its up axis (-Y_cam).
    # The R that maps points old_cam -> new_cam under this rotation is R_y(+dyaw)
    # (verified empirically in Phase 1).
    R = np.array([[np.cos(dyaw_rad), 0.0, np.sin(dyaw_rad)],
                  [0.0,              1.0, 0.0],
                  [-np.sin(dyaw_rad), 0.0, np.cos(dyaw_rad)]], dtype=np.float64)

    # Plane normal in old-camera frame: the assumed flat scene is at +Z = d
    n = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    H_3d = R - np.outer(t, n) / d
    H = K @ H_3d @ K_inv
    return cv2.warpPerspective(frame, H, (w, h))


# Global buffers
video_buffer = VideoBuffer()
twist_buffer = TwistBuffer()

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            # Per-handler cache: avoid re-running depth inference on the same
            # stale frame while latency keeps us serving it for many ticks.
            depth_cache = {'ts': None, 'tensor': None}

            try:
                while True:
                    frame, frame_age_ms, frame_ts = video_buffer.get_frame_with_age()
                    if frame is not None:
                        display = frame.copy()
                        pred_dx = pred_dy = pred_yaw_deg = 0.0
                        mode_label = "off"
                        if video_buffer.predict_enabled and frame_ts is not None and frame_age_ms > 0:
                            pred_dx, pred_dy, dyaw = twist_buffer.integrate(frame_ts, time.time())
                            pred_yaw_deg = np.rad2deg(dyaw)
                            mode_label = "flat"
                            if video_buffer.use_depth:
                                try:
                                    if depth_cache['ts'] != frame_ts:
                                        depth_cache['tensor'] = estimate_depth_tensor(
                                            display, video_buffer.scene_depth_m
                                        )
                                        depth_cache['ts'] = frame_ts
                                    display = predict_view_depth(
                                        display, depth_cache['tensor'],
                                        pred_dx, pred_dy, dyaw, video_buffer.hfov_rad
                                    )
                                    mode_label = "depth"
                                except Exception as e:
                                    print(f"Depth mode failed ({e}); falling back to flat-plane this frame")
                                    display = predict_view(display, pred_dx, pred_dy, dyaw,
                                                           video_buffer.hfov_rad,
                                                           video_buffer.scene_depth_m)
                            else:
                                display = predict_view(display, pred_dx, pred_dy, dyaw,
                                                       video_buffer.hfov_rad,
                                                       video_buffer.scene_depth_m)
                        # Overlay debug info on frame
                        text = f"Latency: {video_buffer.latency_ms}ms | Frame age: {frame_age_ms:.0f}ms"
                        if video_buffer.predict_enabled:
                            text += (f" | Pred[{mode_label}]: yaw {pred_yaw_deg:+.1f}deg,"
                                     f" fwd {pred_dx:+.2f}m, lat {pred_dy:+.2f}m"
                                     f" @ d={video_buffer.scene_depth_m:.1f}m")
                        cv2.putText(display, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        _, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(jpeg.tobytes())
                        self.wfile.write(b'\r\n')
                    time.sleep(0.033)  # ~30 fps
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path.startswith('/latency/'):
            # Set latency: /latency/100 sets 100ms
            try:
                ms = int(self.path.split('/')[-1])
                video_buffer.set_latency(ms)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f'Latency set to {ms}ms\n'.encode())
            except:
                self.send_response(400)
                self.end_headers()

        elif self.path.startswith('/predict/'):
            # Toggle prediction: /predict/1 enables, /predict/0 disables
            try:
                val = int(self.path.split('/')[-1])
                video_buffer.set_predict(val)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f'Prediction {"ON" if val else "OFF"}\n'.encode())
            except:
                self.send_response(400)
                self.end_headers()

        elif self.path.startswith('/depth/'):
            # Toggle per-pixel depth mode: /depth/1 enables, /depth/0 disables
            try:
                val = int(self.path.split('/')[-1])
                video_buffer.set_use_depth(val)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                # set_use_depth may have refused if torch isn't available
                actual = video_buffer.use_depth
                self.wfile.write(
                    f'Depth mode {"ON" if actual else "OFF"}'
                    f'{" (torch unavailable; install pytorch+transformers+pillow)" if val and not actual else ""}\n'.encode()
                )
            except:
                self.send_response(400)
                self.end_headers()

        elif self.path.startswith('/hfov/'):
            # Set camera horizontal FOV in degrees: /hfov/62
            try:
                deg = float(self.path.split('/')[-1])
                if deg <= 0 or deg >= 180:
                    raise ValueError('HFOV must be in (0, 180) degrees')
                video_buffer.set_hfov_deg(deg)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f'HFOV set to {deg} deg\n'.encode())
            except:
                self.send_response(400)
                self.end_headers()

        elif self.path.startswith('/scene_depth/'):
            # Set assumed scene depth in meters: /scene_depth/2.5
            try:
                m = float(self.path.split('/')[-1])
                if m <= 0:
                    raise ValueError('Scene depth must be > 0 m')
                video_buffer.set_scene_depth_m(m)
                self.send_response(200)
                self.send_header('Content-type', 'text/plain')
                self.end_headers()
                self.wfile.write(f'Scene depth set to {m} m\n'.encode())
            except:
                self.send_response(400)
                self.end_headers()

        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            # Snapshot current server state so the UI reflects reality on reload
            cur_latency = video_buffer.latency_ms
            cur_hfov_deg = video_buffer.hfov_rad * 180.0 / np.pi
            cur_scene_depth = video_buffer.scene_depth_m
            predict_checked = 'checked' if video_buffer.predict_enabled else ''
            depth_checked = 'checked' if video_buffer.use_depth else ''
            depth_available_note = '' if DEPTH_AVAILABLE else \
                ' <span style="color:#ff8080;font-size:12px;">(install torch + transformers + pillow)</span>'
            html = f'''<!DOCTYPE html>
<html>
<head>
    <title>Robot Camera</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {{ margin: 0; background: #000; display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 100vh; font-family: sans-serif; }}
        img {{ max-width: 100%; height: auto; }}
        .controls {{ color: white; padding: 20px; text-align: center; }}
        input {{ width: 200px; }}
        button {{ padding: 10px 20px; margin: 5px; font-size: 16px; }}
    </style>
</head>
<body>
    <img src="/video" alt="Robot Camera">
    <div class="controls">
        <p>Artificial Latency: <span id="val">{cur_latency}</span>ms</p>
        <input type="range" id="latency" min="0" max="10000" step="100" value="{cur_latency}">
        <br>
        <button onclick="setLatency(0)">0ms</button>
        <button onclick="setLatency(100)">100ms</button>
        <button onclick="setLatency(200)">200ms</button>
        <button onclick="setLatency(300)">300ms</button>
        <button onclick="setLatency(500)">500ms</button>
        <button onclick="setLatency(1000)">1s</button>
        <button onclick="setLatency(2000)">2s</button>
        <button onclick="setLatency(3000)">3s</button>
        <button onclick="setLatency(5000)">5s</button>
        <button onclick="setLatency(10000)">10s</button>
        <p style="margin-top: 20px;">Predictive Display:
            <label style="margin-left: 10px;">
                <input type="checkbox" id="predict" {predict_checked} onchange="setPredict(this.checked)"> Enabled
            </label>
            <label style="margin-left: 20px;">
                <input type="checkbox" id="usedepth" {depth_checked} onchange="setUseDepth(this.checked)"> Use per-pixel depth (Phase 2b){depth_available_note}
            </label>
        </p>
        <p>Camera HFOV:
            <input type="number" id="hfov" min="10" max="170" step="1" value="{cur_hfov_deg:.1f}" style="width: 80px;" onchange="setHfov(this.value)">
            <span style="margin-left: 5px;">degrees</span>
            <button onclick="setHfov(60)" style="padding: 4px 10px; font-size: 13px;">60</button>
            <button onclick="setHfov(62)" style="padding: 4px 10px; font-size: 13px;">62 (TB3)</button>
            <button onclick="setHfov(80)" style="padding: 4px 10px; font-size: 13px;">80</button>
            <button onclick="setHfov(90)" style="padding: 4px 10px; font-size: 13px;">90</button>
        </p>
        <p>Scene depth (flat-plane proxy):
            <input type="number" id="depth" min="0.2" max="50" step="0.1" value="{cur_scene_depth:.1f}" style="width: 80px;" onchange="setDepth(this.value)">
            <span style="margin-left: 5px;">meters</span>
            <button onclick="setDepth(1)" style="padding: 4px 10px; font-size: 13px;">1m</button>
            <button onclick="setDepth(2)" style="padding: 4px 10px; font-size: 13px;">2m</button>
            <button onclick="setDepth(5)" style="padding: 4px 10px; font-size: 13px;">5m</button>
            <button onclick="setDepth(10)" style="padding: 4px 10px; font-size: 13px;">10m</button>
        </p>
    </div>
    <script>
        const slider = document.getElementById('latency');
        const val = document.getElementById('val');
        const hfovInput = document.getElementById('hfov');
        const depthInput = document.getElementById('depth');
        slider.oninput = () => {{ val.textContent = slider.value; }};
        slider.onchange = () => {{ setLatency(slider.value); }};
        function setLatency(ms) {{
            fetch('/latency/' + ms);
            slider.value = ms;
            val.textContent = ms;
        }}
        function setPredict(enabled) {{
            fetch('/predict/' + (enabled ? 1 : 0));
        }}
        function setUseDepth(enabled) {{
            fetch('/depth/' + (enabled ? 1 : 0));
        }}
        function setHfov(deg) {{
            fetch('/hfov/' + deg);
            hfovInput.value = deg;
        }}
        function setDepth(m) {{
            fetch('/scene_depth/' + m);
            depthInput.value = m;
        }}
    </script>
</body>
</html>'''
            self.wfile.write(html.encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress logging

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('video_server')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.get_logger().info('Subscribed to /camera/image_raw and /cmd_vel')

    def twist_callback(self, msg):
        twist_buffer.add(msg.linear.x, msg.angular.z)

    def callback(self, msg):
        try:
            # Direct conversion without cv_bridge
            if msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'mono8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                # Try generic approach
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                if frame.shape[2] == 4:  # RGBA or BGRA
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            video_buffer.add_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    global ARTIFICIAL_LATENCY_MS

    parser = argparse.ArgumentParser()
    parser.add_argument('--latency', type=int, default=0, help='Initial artificial latency in ms')
    parser.add_argument('--port', type=int, default=8080, help='HTTP server port')
    parser.add_argument('--hfov-deg', type=float, default=62.2,
                        help='Camera horizontal FOV in degrees (default 62.2, TurtleBot3 Gazebo)')
    parser.add_argument('--scene-depth', type=float, default=2.0,
                        help='Assumed flat-plane scene depth in meters (default 2.0)')
    parser.add_argument('--predict', action='store_true', help='Enable predictive display at startup')
    parser.add_argument('--use-depth', action='store_true',
                        help='Enable per-pixel depth mode (Phase 2b, requires torch+transformers+pillow)')
    args = parser.parse_args()

    ARTIFICIAL_LATENCY_MS = args.latency
    video_buffer.set_latency(args.latency)
    video_buffer.set_hfov_deg(args.hfov_deg)
    video_buffer.set_scene_depth_m(args.scene_depth)
    if args.predict:
        video_buffer.set_predict(True)
    if args.use_depth:
        video_buffer.set_use_depth(True)

    rclpy.init()
    node = CameraSubscriber()

    # Start HTTP server in thread
    class ThreadingHTTPServer(ThreadingTCPServer):
        allow_reuse_address = True
    server = ThreadingHTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    print(f'\n=== Video Server Started ===')
    print(f'View at: http://127.0.0.1:{args.port}/')
    print(f'Initial: latency={args.latency}ms  HFOV={args.hfov_deg}deg  '
          f'scene_depth={args.scene_depth}m  predict={"ON" if args.predict else "OFF"}  '
          f'depth={"ON" if video_buffer.use_depth else "OFF"}')
    if not DEPTH_AVAILABLE:
        print(f'Depth mode unavailable (no torch/transformers/pillow); flat-plane only.')
    print(f'Endpoints: /latency/<ms>  /predict/<0|1>  /depth/<0|1>  /hfov/<deg>  /scene_depth/<m>')
    print('============================\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
