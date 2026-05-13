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

# Configurable artificial latency (milliseconds)
ARTIFICIAL_LATENCY_MS = 0

class VideoBuffer:
    """Thread-safe frame buffer with latency injection"""
    def __init__(self):
        self.lock = Lock()
        self.frame_queue = deque()  # (timestamp, frame) pairs
        self.latency_ms = ARTIFICIAL_LATENCY_MS
        self.predict_enabled = False
        self.hfov_rad = 1.085  # ~62 deg, typical TurtleBot3 Gazebo camera
        self.scene_depth_m = 2.0  # assumed flat-plane scene distance (m)

    def set_latency(self, ms):
        with self.lock:
            self.latency_ms = ms
            print(f"Latency set to {ms}ms, buffer has {len(self.frame_queue)} frames")

    def set_predict(self, enabled):
        with self.lock:
            self.predict_enabled = bool(enabled)
            print(f"Prediction {'ON' if self.predict_enabled else 'OFF'}")

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

            try:
                while True:
                    frame, frame_age_ms, frame_ts = video_buffer.get_frame_with_age()
                    if frame is not None:
                        display = frame.copy()
                        pred_yaw_deg = 0.0
                        pred_dx = pred_dy = pred_yaw_deg = 0.0
                        if video_buffer.predict_enabled and frame_ts is not None and frame_age_ms > 0:
                            pred_dx, pred_dy, dyaw = twist_buffer.integrate(frame_ts, time.time())
                            pred_yaw_deg = np.rad2deg(dyaw)
                            display = predict_view(display, pred_dx, pred_dy, dyaw,
                                                   video_buffer.hfov_rad,
                                                   video_buffer.scene_depth_m)
                        # Overlay debug info on frame
                        text = f"Latency: {video_buffer.latency_ms}ms | Frame age: {frame_age_ms:.0f}ms"
                        if video_buffer.predict_enabled:
                            text += (f" | Pred: yaw {pred_yaw_deg:+.1f}deg,"
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
        <p style="margin-top: 20px;">Predictive Display (yaw + forward/lateral, flat-plane):
            <label style="margin-left: 10px;">
                <input type="checkbox" id="predict" {predict_checked} onchange="setPredict(this.checked)"> Enabled
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
    args = parser.parse_args()

    ARTIFICIAL_LATENCY_MS = args.latency
    video_buffer.set_latency(args.latency)
    video_buffer.set_hfov_deg(args.hfov_deg)
    video_buffer.set_scene_depth_m(args.scene_depth)
    if args.predict:
        video_buffer.set_predict(True)

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
          f'scene_depth={args.scene_depth}m  predict={"ON" if args.predict else "OFF"}')
    print(f'Endpoints: /latency/<ms>  /predict/<0|1>  /hfov/<deg>  /scene_depth/<m>')
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
