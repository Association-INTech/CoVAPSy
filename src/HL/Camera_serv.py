import io
import threading
import logging
from http import server
import socketserver
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
from threading import Condition

# -------------------------------------------------------------------
# Stockage global de la dernière frame reçue
# -------------------------------------------------------------------
class FrameBuffer:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def update(self, data):
        with self.condition:
            self.frame = data
            self.condition.notify_all()

    def get(self):
        # Ça te donne la dernière frame (ou None au début)
        return self.frame


frame_buffer = FrameBuffer()


# -------------------------------------------------------------------
# Output pour Picamera2 (reçoit les JPEG du JpegEncoder)
# -------------------------------------------------------------------
class StreamOutput(io.BufferedIOBase):
    def write(self, buf):
        frame_buffer.update(buf)


# -------------------------------------------------------------------
# Serveur MJPEG : /stream.mjpg
# -------------------------------------------------------------------
class StreamHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            try:
                while True:
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
        else:
            self.send_error(404)


class StreamServer(socketserver.ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# -------------------------------------------------------------------
# Thread principal de capture + serveur HTTP
# -------------------------------------------------------------------
def start_camera_stream(port=8000, size=(640, 480)):
    def run():
        picam2 = Picamera2()
        picam2.configure(picam2.create_video_configuration(
            main={"size": size}
        ))

        output = StreamOutput()
        picam2.start_recording(JpegEncoder(), FileOutput(output))

        httpd = StreamServer(("", port), StreamHandler)
        print(f"[INFO] Serveur MJPEG en ligne sur http://<IP>:{port}/stream.mjpg")

        try:
            httpd.serve_forever()
        finally:
            picam2.stop_recording()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


# -------------------------------------------------------------------
# Fonction utilitaire pour d’autres scripts
# -------------------------------------------------------------------
def get_current_frame():
    """Retourne la dernière frame JPEG"""
    return frame_buffer.get()
