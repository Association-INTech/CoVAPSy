from picamera2 import Picamera2, Preview  # type: ignore
from PIL import Image  # For saving images
import time
import os
import shutil 
# Initialize the camera

# Set LIBCAMERA_LOG_LEVELS to suppress INFO logs
os.environ["LIBCAMERA_LOG_LEVELS"] = "WARN"


picam2 = Picamera2()

def run_test(resolution):
    # Configure the camera for preview
    config = picam2.create_preview_configuration(main={"size": resolution}) # full sensor resolution (3280, 2464)
    picam2.configure(config)

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
    total_array_time = 0
    total_convert_time = 0
    total_path_creation_time = 0
    total_save_time = 0

    Start_time = time.time()  # Start time for the image stream

    for i in range(N):
        init_time = time.time()  # Initialize the time for the first frame

        # Capture the frame
        frame = picam2.capture_array()
        capture_time = time.time()  # Time after capturing the frame
        total_capture_time += (capture_time - init_time)

        # Convert the frame to an image
        image = Image.fromarray(frame)
        array_time = time.time()  # Time after converting the frame to an image
        total_array_time += (array_time - capture_time)

        # Convert the image to RGB
        #image = image.convert("RGB")
        convert_time = time.time()  # Time after converting the image to RGB
        total_convert_time += (convert_time - array_time)

        # Create the file path
        #frame_path = os.path.join(save_dir, f"frame_{frame_count:04d}.jpg")
        path_creation_time = time.time()  # Time after creating the path
        total_path_creation_time += (path_creation_time - convert_time)

        # Save the image
        #image.save(frame_path)
        save_time = time.time()  # Time after saving the image
        total_save_time += (save_time - path_creation_time)

        frame_count += 1
    picam2.stop()  # Stop the camera
    # Calculate averages
    average_capture_time = total_capture_time / N
    average_array_time = total_array_time / N
    average_convert_time = total_convert_time / N
    average_path_creation_time = total_path_creation_time / N
    average_save_time = total_save_time / N
    total_time = time.time() - Start_time

    # Print results
    print(f"Average Capture time: {average_capture_time:.5f} seconds")
    print(f"Average Array time: {average_array_time:.5f} seconds")
    print(f"Average Convert time: {average_convert_time:.5f} seconds")
    print(f"Average Path creation time: {average_path_creation_time:.5f} seconds")
    print(f"Average Save time: {average_save_time:.5f} seconds")
    print(f"Total time for {N} iterations: {total_time:.5f} seconds. FPS: {N / total_time:.2f}")
    
    
if __name__ == "__main__":
    N = 100  # Number of iterations to measure
    
    for resolution in [(3280, 2464),  (1280, 720), (1920, 1080),(640, 480), (320, 240)]:
        print(f"Testing resolution: {resolution}")
        run_test(resolution)