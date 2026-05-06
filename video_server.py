#!/usr/bin/env python3
"""
MJPEG video server with artificial latency injection.
View at http://<IP>:8080/video in Quest browser.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

    def set_latency(self, ms):
        with self.lock:
            self.latency_ms = ms
            print(f"Latency set to {ms}ms, buffer has {len(self.frame_queue)} frames")

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
        """Returns (frame, age_in_ms)"""
        with self.lock:
            if not self.frame_queue:
                return None, 0

            now = time.time()
            delay_sec = self.latency_ms / 1000.0
            target_time = now - delay_sec

            # If latency is 0, return newest frame
            if self.latency_ms == 0:
                ts, frame = self.frame_queue[-1]
                return frame, (now - ts) * 1000

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
            return best_frame, age_ms

# Global buffer
video_buffer = VideoBuffer()

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
                    frame, frame_age_ms = video_buffer.get_frame_with_age()
                    if frame is not None:
                        # Overlay debug info on frame
                        display = frame.copy()
                        text = f"Latency: {video_buffer.latency_ms}ms | Frame age: {frame_age_ms:.0f}ms"
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

        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
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
        <p>Artificial Latency: <span id="val">{ARTIFICIAL_LATENCY_MS}</span>ms</p>
        <input type="range" id="latency" min="0" max="10000" step="100" value="{ARTIFICIAL_LATENCY_MS}">
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
    </div>
    <script>
        const slider = document.getElementById('latency');
        const val = document.getElementById('val');
        slider.oninput = () => {{ val.textContent = slider.value; }};
        slider.onchange = () => {{ setLatency(slider.value); }};
        function setLatency(ms) {{
            fetch('/latency/' + ms);
            slider.value = ms;
            val.textContent = ms;
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
        self.get_logger().info('Subscribed to /camera/image_raw')

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
    args = parser.parse_args()

    ARTIFICIAL_LATENCY_MS = args.latency
    video_buffer.set_latency(args.latency)

    rclpy.init()
    node = CameraSubscriber()

    # Start HTTP server in thread
    class ThreadingHTTPServer(ThreadingTCPServer):
        allow_reuse_address = True
    server = ThreadingHTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    import socket
    ip = socket.gethostbyname(socket.gethostname())
    print(f'\n=== Video Server Started ===')
    print(f'View at: http://{ip}:{args.port}/')
    print(f'Initial latency: {args.latency}ms')
    print(f'Adjust via slider or /latency/<ms> endpoint')
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
