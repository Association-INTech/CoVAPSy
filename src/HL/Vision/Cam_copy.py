from picamera2 import Picamera2
import time
import os

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (640, 480)}, buffer_count=4)
picam2.configure(config)

# Directory for saving images
save_dir = "captured_frames"
os.makedirs(save_dir, exist_ok=True)

# Start capturing
picam2.start()

frame_count = 0
start_time = time.time()
capture_time = 5  # Capture for 5 seconds

while time.time() - start_time < capture_time:
    filename = os.path.join(save_dir, f"frame_{frame_count:04d}.jpg")
    picam2.capture_file(filename, format="jpeg")
    frame_count += 1

picam2.stop()

# Performance report
elapsed_time = time.time() - start_time
fps = frame_count / elapsed_time
print(f"Captured {frame_count} frames in {elapsed_time:.2f} seconds ({fps:.2f} FPS)")
