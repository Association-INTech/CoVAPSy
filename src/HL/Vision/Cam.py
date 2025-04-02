from picamzero import Camera # type: ignore #ignore the module could not be resolved error because it is a rpi only module
import os
import time
Startime = time.time()
home_dir = os.environ['HOME'] #set the location of your home directory
cam = Camera()
init_time = time.time() - Startime

cam.take_photo(f"{home_dir}/CoVAPSy/src/HL/Vision/Captured_image/new_image.jpg") #save the image to your desktop
photo_time = time.time() - Startime
print(f"Photo taken in {photo_time:.5f} seconds")
print(f"Init time: {init_time:.5f} seconds")
print(f"Total time: {photo_time + init_time:.5f} seconds")