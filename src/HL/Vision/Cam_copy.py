from picamera2 import Picamera2, Preview  # type: ignore
from PIL import Image  # For saving images
import time
import os
import shutil 
from pprint import *
# Initialize the camera

# Set LIBCAMERA_LOG_LEVELS to suppress INFO logs
os.environ["LIBCAMERA_LOG_LEVELS"] = "WARN"



def run_test():
    # Initialize the camera
    picam2 = Picamera2()

    capture_config = picam2.create_still_configuration()

    # Start the camera
    picam2.start()

    # Parameters
    
    frame_count = 0  # Counter to keep track of saved frames
    save_dir = "Captured_Frames"  # Directory to save frames
    if os.path.exists(save_dir):
        shutil.rmtree(save_dir)  # Remove the directory and its contents
    os.makedirs(save_dir, exist_ok=True)  # Create the directory if it doesn't exist

    # Initialize timing accumulators
    total_capture_time = 0

    Start_time = time.time()  # Start time for the image stream

    for i in range(N):
        init_time = time.time()  # Initialize the time for the first frame

        # Capture the frame
        picam2.switch_mode_and_capture_file(capture_config, f"frame_{frame_count:04d}.jpg")
        capture_time = time.time()  # Time after capturing the frame
        total_capture_time += capture_time - init_time
        
        frame_count += 1
    picam2.stop()  # Stop the camera
    # Calculate averages
    average_capture_time = total_capture_time / N
    total_time = time.time() - Start_time

    # Print results
    print(f"Average Capture time: {average_capture_time:.5f} seconds")
    print(f"Total time for {N} iterations: {total_time:.5f} seconds. FPS: {N / total_time:.2f}")
    
    
if __name__ == "__main__":
    N=10
    run_test()