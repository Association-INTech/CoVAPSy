from picamera2 import Picamera2, Preview # type: ignore #ignore the module could not be resolved error because it is a rpi only module
from PIL import Image  # For saving images
import time
import os
import threading

# Initialize the camera
picam2 = Picamera2()

# Configure the camera for preview
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)

# Start the camera
picam2.start()


Start_time = time.time()  # Start time for the image stream


frame_count = 0  # Counter to keep track of saved frames
save_dir = "Captured_Frames"  # Directory to save frames

os.makedirs(save_dir, exist_ok=True)  # Create the directory if it doesn't exist

init_time = time.time()  # Initialize the time for the first frame
        
frame = picam2.capture_array()
capture_time = time.time()  # Time after capturing the frame
image = Image.fromarray(frame)
array_time = time.time()  # Time after converting the frame to an image
image = image.convert("RGB")
convert_time = time.time()  # Time after converting the image to RGB
frame_path = os.path.join(save_dir, f"frame_{frame_count:04d}.jpg")
path_creation_time = time.time()  # Time after creating the path
image.save(frame_path)
save_time = time.time()  # Time after saving the image

print(f"Initialization time: {init_time - Start_time:.5f} seconds")
print(f"Capture time: {capture_time - init_time:.5f} seconds")
print(f"Array time: {array_time - capture_time:.5f} seconds")
print(f"Convert time: {convert_time - array_time:.5f} seconds")
print(f"Path creation time: {path_creation_time - convert_time:.5f} seconds")
print(f"Save time: {save_time - path_creation_time:.5f} seconds")
print(f"Total time: {save_time - Start_time:.5f} seconds")



       

