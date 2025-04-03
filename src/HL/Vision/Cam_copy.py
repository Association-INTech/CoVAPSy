from picamera2 import Picamera2
import time
import os

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "YUV420"})
picam2.configure(config)

# Directory for saving images
save_dir = "captured_frames"
os.makedirs(save_dir, exist_ok=True)

frame_count = 0
start_time = time.time()
capture_time = 5  # Capture for 5 seconds

# Start continuous capture
picam2.start_and_capture_files(os.path.join(save_dir, "frame_{:04d}.jpg"), format="jpeg", num_files=200)

picam2.stop()

# Performance report
elapsed_time = time.time() - start_time
fps = frame_count / elapsed_time
print(f"Captured {frame_count} frames in {elapsed_time:.2f} seconds ({fps:.2f} FPS)")
