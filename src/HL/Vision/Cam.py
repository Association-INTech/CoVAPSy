from picamzero import Camera # type: ignore #ignore the module could not be resolved error because it is a rpi only module
import os

home_dir = os.environ['HOME'] #set the location of your home directory
cam = Camera()

cam.start_preview()
cam.take_photo(f"{home_dir}/CoVAPSy/src/HL/Vision/Captured_image/new_image.jpg") #save the image to your desktop
cam.stop_preview()