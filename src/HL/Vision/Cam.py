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
Capture_time = time.time() - init_time  # Time taken to capture the first frame
image = Image.fromarray(frame)
Array_time = time.time() - Capture_time  # Time taken to convert the frame to an image
image = image.convert("RGB")
convert_time = time.time() - Array_time  # Time taken to convert the image to RGB
frame_path = os.path.join(save_dir, f"frame_{frame_count:04d}.jpg")
Path_creation_time = time.time() - convert_time  # Time taken to create the path
image.save(frame_path)
save_time = time.time() - Path_creation_time  # Time taken to save the image

print(f"Initialization time: {init_time - Start_time:.5f} seconds")
print(f"Capture time: {init_time - Capture_time:.5f} seconds")
print(f"Array time: {Capture_time - Array_time:.5f} seconds")
print(f"Convert time: {Array_time - convert_time:.5f} seconds")
print(f"Path creation time: {convert_time - Path_creation_time:.5f} seconds")
print(f"Save time: {Path_creation_time - save_time:.5f} seconds")
print(f"Total time: {time.time() - Start_time:.5f} seconds")



       

