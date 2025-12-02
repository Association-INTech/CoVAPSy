# camera_server.py
import io
import logging
from http import server
import socketserver
from threading import Condition

streaming_enabled = True
from Camera_serv import streaming_enabled
class FrameBuffer:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def update(self, data):
        with self.condition:
            self.frame = data
            self.condition.notify_all()

    def get(self):
        return self.frame


frame_buffer = FrameBuffer()


class StreamOutput(io.BufferedIOBase):
    def write(self, buf):
        frame_buffer.update(buf)


class StreamHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            try:
                try:
                    while streaming_enabled:
                        with frame_buffer.condition:
                            frame_buffer.condition.wait()
                            frame = frame_buffer.frame

                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", len(frame))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")

                except Exception as e:
                    logging.warning("Client déconnecté: %s", e)

            except Exception as e:
                logging.warning("Client déconnecté: %s", e)
        else:
            self.send_error(404)


class StreamServer(socketserver.ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True
