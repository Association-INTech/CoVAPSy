# camera_server.py
import io
import logging
from http import server
import socketserver
from threading import Condition
streaming_enabled = True

from src.HL.programme.Camera_serv import streaming_enabled

class FrameBuffer:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def update(self, data):
        # Only accept full JPEG frames
        if data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'):
            with self.condition:
                self.frame = data
                self.condition.notify_all()

    def get(self):
        return self.frame


frame_buffer = FrameBuffer()


class StreamOutput(io.BufferedIOBase):
    def write(self, buf):
        frame_buffer.update(buf)
        return len(buf)


class StreamHandler(server.BaseHTTPRequestHandler):
    def __init__(self):
        self.log = logging.getLogger(__name__)
    
    def log_message(self, format, *args):
        logging.getLogger(__name__).info(format % args)
        
    def do_GET(self):
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        try:
            while streaming_enabled:
                with frame_buffer.condition:
                    frame_buffer.condition.wait()
                    frame = frame_buffer.frame

                if frame is None:
                    continue

                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", len(frame))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")

        except Exception as e:
            self.log.warning("Client disconnected: %s", e)


class StreamServer(socketserver.ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True
