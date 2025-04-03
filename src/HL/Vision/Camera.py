from picamera2 import Picamera2 # type: ignore
from PIL import Image
import numpy as np
import os
import logging

N_IMAGES = 100  # Number of images to capture
SAVE_DIR = "Captured_Frames"  # Directory to save frames
COLOUR_KEY = {
    "green": 1,
    "red": -1,
    "none": 0
}
COLOR_THRESHOLD = 20  # Threshold for color intensity difference
Y_OFFSET = 0.5  # Offset for the y-axis in the image

class Camera:
    def __init__(self):
        self.image_no = 0
        self.image_path = None
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(main={"size": (1920, 1080)}, format= "BGR888")
        self.camera.configure(config)
        self.camera.start()
        self.flag_stop = False
        
    def capture_image(self):
        
        self.image_path = os.path.join(SAVE_DIR, f"frame_{self.image_no:02d}.jpg")
        frame = self.camera.capture_array()
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
    
    def stop(self):
        self.flag_stop = True
        self.camera.stop()
        self.camera.close()
        
    def get_last_image(self):
        last_image_no= self.image_no - 1 if self.image_no > 0 else 99 # 
        image_path = os.path.join(SAVE_DIR, f"frame_{last_image_no:02d}.jpg")
        return Image.open(image_path) #.convert("RGB")
    
    def camera_matrix(self, image, vector_size=128):
        """
        Create a matrix of -1, 0, and 1 for a line in the image. The matrix size is 128.
        """
        height, width, _ = image.shape
        if vector_size > width:
            raise ValueError("Vector size cannot be greater than image width")

        # Slice the middle 5% of the image height
        sliced_image = image[height // 2 - height // 40 : height // 2 + height // 40, :, :]

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
        if logging.getLogger().isEnabledFor(logging.DEBUG):
            self.recreate_image_from_matrix(sliced_image, output_matrix, adjusted_width, vector_size)

        return output_matrix
    
    def recreate_image_from_matrix(image, matrix,adjusted_width, vector_size=128):
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
        Image.fromarray(combined_image).convert("RGB").save(f"debug_combined_image{counter}.jpg")