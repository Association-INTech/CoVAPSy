from picamera2 import Picamera2 # type: ignore
from PIL import Image
import numpy as np
import os
import logging as log
import threading
import shutil
import scipy as sp

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

class Camera:
    def __init__(self):
        os.environ["LIBCAMERA_LOG_LEVELS"] = "WARN"
        self.debug_counter = 0  # Counter for debug images
        self.image_no = 0
        self.image_path = None
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (1920, 1080)})
        self.picam2.configure(config)
        self.picam2.start()
        self.flag_stop = False
        self.thread = None  # Stocke le thread pour contrôle ultérieur
        picamera2_logger = log.getLogger("picamera2")
        picamera2_logger.setLevel(log.INFO)
        os.makedirs(SAVE_DIR, exist_ok=True)  # Crée le répertoire s'il n'existe pas
        os.makedirs(DEBUG_DIR, exist_ok=True)  # Crée le répertoire de débogage s'il n'existe pas
        os.makedirs(DEBUG_DIR_wayfinding, exist_ok=True)  # Crée le répertoire de débogage s'il n'existe pas
        self.capture_image()  # Capture une image pour initialiser le répertoire de sauvegarde
        
    def capture_image(self):
        
        self.image_path = os.path.join(SAVE_DIR, f"frame_{self.image_no:02d}.jpg")
        frame = self.picam2.capture_array()
        image = Image.fromarray(frame).convert("RGB")
        image.save(self.image_path)
        self.image_no += 1
        return image
    
    def capture_images_continus(self):
        while True:
            for i in range(N_IMAGES):
                self.capture_image()
                if self.flag_stop:
                    break
            if self.flag_stop:
                break
            self.image_no = 0

    def start(self):
        """Démarre la capture en continu dans un thread séparé."""
        if self.thread is None or not self.thread.is_alive():  # Évite de lancer plusieurs threads
            self.flag_stop = False  # Réinitialiser le flag d'arrêt
            self.thread = threading.Thread(target=self.capture_images_continus, daemon=True)
            self.thread.start()
    
    def stop(self):
        """Arrête la capture et attend que le thread se termine proprement."""
        self.flag_stop = True
        if self.thread is not None:
            self.thread.join()  # Attendre la fin du thread avant de continuer
        self.picam2.stop()
        self.picam2.close()
        shutil.rmtree(SAVE_DIR)  # Supprime le répertoire des images capturées
        
    def get_last_image(self):
        last_image_no= self.image_no - 1 if self.image_no > 0 else 99 # 
        image_path = os.path.join(SAVE_DIR, f"frame_{last_image_no:02d}.jpg")
        image= Image.open(image_path).convert("RGB")
        image_np = np.array(image)
        return image_np
    
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
    image_path = "src\HL\wrong_direction33.jpg"  # Replace with your image path
    camera = Camera()
    camera.start()
    pil_image = Image.open(image_path).convert("RGB")  # Open and ensure it's in RGB format
    image = np.array(pil_image)  # Convert to NumPy array
    
    print(camera.is_running_in_reversed(image=image, LEFT_IS_GREEN=True))  # Check if the car is running in reverse