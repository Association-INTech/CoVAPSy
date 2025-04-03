from picamera2 import Picamera2, NullPreview
import time
import os

# Initialize Picamera2 with NullPreview
picam2 = Picamera2()
picam2.preview_configuration = NullPreview(picam2)

# Configure the camera
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)

# Directory for saving images
save_dir = "captured_frames"
os.makedirs(save_dir, exist_ok=True)

# Start capturing images
frame_count = 10
start_time = time.time()

picam2.start_and_capture_files(os.path.join(save_dir, "test{:d}.jpg"), num_files=frame_count)

# Performance report
elapsed_time = time.time() - start_time
fps = frame_count / elapsed_time
print(f"Captured {frame_count} frames in {elapsed_time:.2f} seconds ({fps:.2f} FPS)")

picam2.stop()