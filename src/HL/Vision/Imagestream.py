from picamera2 import Picamera2, Preview # type: ignore #ignore the module could not be resolved error because it is a rpi only module
import cv2 # type: ignore
import time
import os

def main():
    # Initialize the camera
    picam2 = Picamera2()

    # Configure the camera for preview
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)

    # Start the camera
    picam2.start()

   
    time=time.time()  # Start time for the image stream
    try:
        frame_count = 0  # Counter to keep track of saved frames
        save_dir = "Captured_Frames"  # Directory to save frames
        os.makedirs(save_dir, exist_ok=True)  # Create the directory if it doesn't exist
    
        while True:
            # Capture a frame
            frame = picam2.capture_array()
    
            # Save the frame to disk
            frame_path = os.path.join(save_dir, f"frame_{frame_count:04d}.jpg")
            cv2.imwrite(frame_path, frame)
            frame_count += 1
    
            # Display the frame in a window
            # cv2.imshow("Image Stream", frame)
    
            # Exit the stream when 'q' is pressed
            if time.time() - time > 1:
                print(f"Captured {frame_count} frames in {time.time() - time:.2f} seconds")
                time = time.time()
                frame_count = 0
                time = time.time()
    finally:
        # Stop the camera and close the window
        picam2.stop()

if __name__ == "__main__":
    main()