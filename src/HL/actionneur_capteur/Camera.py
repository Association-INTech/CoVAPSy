import cv2
from picamera2 import Picamera2 # type: ignore
from PIL import Image
import numpy as np
import os
import logging as log
import threading
import shutil
import scipy as sp
import time
import logging

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
N_IMAGES = 100  # Number of images to capture
SAVE_DIR = "Captured_Frames"  # Directory to save frames
DEBUG_DIR = "Debug"  # Directory for debug images
DEBUG_DIR_wayfinding = "Debug_Wayfinding"  # Directory for wayfinding debug images
COLOUR_KEY = {
    "green": 1,
    "red": -1,
    "none": 0
}
COLOR_THRESHOLD = 20  # Threshold for color intensity difference
Y_OFFSET = -80  # Offset for the y-axis in the image

from picamera2.outputs import Output

class JpegCallback(Output):
    def __init__(self, parent_cam):
        super().__init__()
        self.parent = parent_cam

    def outputframe(self, frame, keyframe=True):
        # frame = bytes JPEG
        self.parent._on_new_frame(frame)




from src.HL.programme.Camera_serv import StreamServer, StreamHandler, StreamOutput, frame_buffer
from src.HL.programme.programme import Program
from src.HL.Autotech_constant import PORT_STREAMING_CAMERA, SIZE_CAMERA_X, SIZE_CAMERA_Y, FRAME_RATE, CAMERA_QUALITY, STREAM_PATH

class ProgramStreamCamera(Program):
    def __init__(self,serveur):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
        self.running = False
        self.controls_car = False
    
    @property
    def camera(self):
        # accès dynamique
        return self.serveur.camera

    
    def start(self):
        cam = self.camera
        if cam is None:
            self.log.error("Camera not initialized yet")
            return
        
        self.running = True
        self.camera.start_stream()
    
    def kill(self):
        self.running = False
        self.camera.stop_stream()




class Camera:
    def __init__(self, size=(SIZE_CAMERA_X, SIZE_CAMERA_Y), port=PORT_STREAMING_CAMERA):
        self.size = size
        self.port = port

        self.streaming = False
        self.stream_thread = None
        self.picam2 = None

        self.last_frame = None
        self.debug_counter = 0
        self.image_no = 0


        # Démarrage en mode "acquisition locale sans stream"
        self._start_local_capture()


    # ----------------------------------------------------------
    # Capture locale (sans MJPEG server)
    # ----------------------------------------------------------
    def _start_local_capture(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": self.size},     # plus large, moins zoomé
            controls={"FrameRate": FRAME_RATE}       # FPS stable
        )

        self.picam2.configure(config)
        self.output = StreamOutput()

        # Qualité JPEG custom
        self.picam2.start_recording(JpegEncoder(q=CAMERA_QUALITY), FileOutput(self.output))

        # thread lecture last_frame
        self.capture_thread = threading.Thread(
            target=self._update_last_frame_loop,
            daemon=True
        )
        self.capture_thread.start()


    def _update_last_frame_loop(self):
        """Récupère en continu la dernière frame JPEG."""
        while True:
            jpeg = frame_buffer.get()
            if jpeg:
                np_frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if np_frame is not None:
                    self.last_frame = cv2.cvtColor(np_frame, cv2.COLOR_BGR2RGB)
            time.sleep(0.01)


    # ----------------------------------------------------------
    # Contrôle streaming MJPEG
    # ----------------------------------------------------------
    def start_stream(self):
        if self.streaming:
            return
        import src.HL.programme.Camera_serv
        src.HL.programme.Camera_serv.streaming_enabled = True

        self.httpd = StreamServer(("", self.port), StreamHandler)

        def run_server():
            print(f"[INFO] MJPEG stream on http://<IP>:{self.port}/{STREAM_PATH}.mjpg")
            try:
                self.httpd.serve_forever()
            except Exception as e:
                print("Serveur MJPEG arrêté:", e)

        self.stream_thread = threading.Thread(target=run_server, daemon=True)
        self.stream_thread.start()
        self.streaming = True



    def stop_stream(self):
        if not self.streaming:
            return

        import src.HL.programme.Camera_serv
        src.HL.programme.Camera_serv.streaming_enabled = False

        print("[INFO] Shutting down MJPEG server...")

        self.httpd.shutdown()
        self.httpd.server_close()
        self.stream_thread.join()

        self.streaming = False
        print("[INFO] Stream stopped.")




    def toggle_stream(self):
        if self.streaming:
            print("[INFO] Stopping stream")
            self.stop_stream()
        else:
            print("[INFO] Starting stream")
            self.start_stream()


    # ----------------------------------------------------------
    # Interface publique
    # ----------------------------------------------------------
    def get_last_image(self):
        return self.last_frame

    
    def camera_matrix(self, vector_size=128, image=None):
        """
        Create a matrix of -1, 0, and 1 for a line in the image. The matrix size is 128.
        """
        if image is None:
            image = self.get_last_image()
        height, width, _ = image.shape
        if vector_size > width:
            raise ValueError("Vector size cannot be greater than image width")

        # Slice the middle 5% of the image height
        sliced_image = image[height // 2 - height // 40 + Y_OFFSET: height // 2 + height // 40 + Y_OFFSET, :, :]

        # Ensure the width of the sliced image is divisible by vector_size
        adjusted_width = (width // vector_size) * vector_size
        sliced_image = sliced_image[:, :adjusted_width, :]

        # Initialize the output matrix
        output_matrix = np.zeros(vector_size, dtype=int)
        bucket_size = adjusted_width // vector_size

        # Calculate red and green intensities for all segments at once
        reshaped_red = sliced_image[:, :, 0].reshape(sliced_image.shape[0], vector_size, bucket_size)
        reshaped_green = sliced_image[:, :, 1].reshape(sliced_image.shape[0], vector_size, bucket_size)
        red_intensities = np.mean(reshaped_red, axis=(0, 2))
        green_intensities = np.mean(reshaped_green, axis=(0, 2))

        # Determine the color for each segment
        output_matrix[red_intensities > green_intensities + COLOR_THRESHOLD] = COLOUR_KEY["red"]
        output_matrix[green_intensities > red_intensities + COLOR_THRESHOLD] = COLOUR_KEY["green"]
        output_matrix[np.abs(red_intensities - green_intensities) <= COLOR_THRESHOLD] = COLOUR_KEY["none"]

        # Recreate the image from the matrix
        if log.getLogger().isEnabledFor(log.DEBUG):
            path= os.path.join(DEBUG_DIR, f"debug_combined_image{self.debug_counter}.jpg")
            self.recreate_image_from_matrix(sliced_image, output_matrix, adjusted_width, vector_size).save(path)
            

        return output_matrix
    
    def recreate_image_from_matrix(self, image, matrix, adjusted_width, vector_size=128):
        """
        Recreate an image from the matrix of -1, 0, and 1 and append it to the bottom of the sliced image.
        """

        # Create a blank image (20 pixels high)
        recreated_image = np.zeros((20, vector_size, 3), dtype=np.uint8)
        recreated_image[:, matrix == COLOUR_KEY["red"], :] = [255, 0, 0]  # Red
        recreated_image[:, matrix == COLOUR_KEY["green"], :] = [0, 255, 0]  # Green
        recreated_image[:, matrix == COLOUR_KEY["none"], :] = [128, 128, 128]  # Gray

        # Resize the recreated image to match the width of the sliced image
        scale_factor = adjusted_width // vector_size
        recreated_image_resized = np.repeat(recreated_image, scale_factor, axis=1)

        # Adjust the width of the recreated image to match the sliced image
        if recreated_image_resized.shape[1] > adjusted_width:
            recreated_image_resized = recreated_image_resized[:, :adjusted_width, :]
        elif recreated_image_resized.shape[1] < adjusted_width:
            padding = adjusted_width - recreated_image_resized.shape[1]
            recreated_image_resized = np.pad(
                recreated_image_resized,
                ((0, 0), (0, padding), (0, 0)),
                mode="constant",
                constant_values=0,
            )
            recreated_image_resized[:, -padding:, 2] = 255  # Blue channel for padding

        # Append the recreated image to the bottom of the sliced image
        combined_image = np.vstack((image, recreated_image_resized))
        self.debug_counter += 1
        return Image.fromarray(combined_image).convert("RGB")
        
    def is_green_or_red(self,lidar):
        """
        Check if the car is facing a green or red wall by analyzing the bottom half of the image.
        """
        image = self.get_last_image()
        height, _, _ = image.shape
        bottom_half = image[height // 2:, :, :]  # Slice the bottom half of the image
        lidar= np.max(sp.ndimage.zoom(lidar[595:855], image.shape[1]/len(lidar[595:855]),mode="nearest")[None,:],0) # Resize lidar data to match the image size
        print((lidar < 0.5).sum())
        print(f"min lidar: {lidar.min()}, max lidar: {lidar.max()}")
        red_intensity = np.mean(bottom_half[:, :, 0]*(lidar < 0.5))  # Red channel in RGB
        green_intensity = np.mean(bottom_half[:, :, 1]*(lidar < 0.5))  # Green channel in RGB

        if green_intensity > red_intensity + COLOR_THRESHOLD:
            return COLOUR_KEY["green"]
        elif red_intensity > green_intensity + COLOR_THRESHOLD:
            return COLOUR_KEY["red"]
        return COLOUR_KEY["none"]
    
    def is_running_in_reversed(self, image = None, LEFT_IS_GREEN=True):
        """
        Check if the car is running in reverse.
        If the car is in reverse, green will be on the right side of the image and red on the left.
        """
        if image is None:
            image = self.get_last_image()
        matrix = self.camera_matrix(image=image)
        if COLOUR_KEY["green"] not in matrix or COLOUR_KEY["red"] not in matrix:
            # If there are no green or no red pixels, return False
            return False
        green_indices = (matrix == COLOUR_KEY["green"]) * np.arange(1, len(matrix) + 1)
        average_green_index = np.mean(green_indices[green_indices > 0])  # Average index of green

        red_indices = (matrix == COLOUR_KEY["red"]) * np.arange(1, len(matrix) + 1)
        average_red_index = np.mean(red_indices[red_indices > 0])  # Average index of redcolor is red
        
        if LEFT_IS_GREEN and average_red_index > average_green_index:
            if log.getLogger().isEnabledFor(log.DEBUG):
                log.debug(f"green: {average_green_index}, red: {average_red_index}")
                vector_size = 128   
                self.debug_counter += 1
                height, width, _ = image.shape
                sliced_image = image[height // 2 - height // 40 + Y_OFFSET: height // 2 + height // 40 + Y_OFFSET, :, :]

                # Ensure the width of the sliced image is divisible by vector_size
                adjusted_width = (width // vector_size) * vector_size
                sliced_image = sliced_image[:, :adjusted_width, :]
                debug_slice_image=self.recreate_image_from_matrix(sliced_image, matrix, adjusted_width, vector_size)
                
                debug_slice_image.save(os.path.join(DEBUG_DIR_wayfinding, f"wrong_direction_{self.debug_counter}_slice.jpg"))
                Image.fromarray(image).convert("RGB").save(os.path.join(DEBUG_DIR_wayfinding, f"wrong_direction{self.debug_counter}.jpg"))
            return True
        elif not LEFT_IS_GREEN and average_green_index > average_red_index:
            return True

if __name__ == "__main__":
    log.basicConfig(level=log.DEBUG)

    camera = Camera()

    print("Attente frame...")
    while camera.get_last_image() is None:
        time.sleep(0.05)

    frame = camera.get_last_image()
    matrix = camera.camera_matrix()
    print("camera_matrix OK")

    input("Appuyer pour lancer le stream...")
    camera.toggle_stream()

    while True:
        if input("Toggle ? ") == "o":
            camera.toggle_stream()
