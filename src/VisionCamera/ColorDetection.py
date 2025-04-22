import cv2 as cv
import numpy as np


URL = "udp://<192.168.1.10>:8554"

cap = cv.VideoCapture(URL)
while(True):
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # define range of blue color in HSV

    # upper and lower bounds for red color
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # upper and lower bounds for green color
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([90, 255, 255])

    # Threshold the HSV image to get only blue colors

    # Create a red mask
    mask1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv.inRange(hsv, lower_red2, upper_red2)
    
    # Bitwise-OR the masks
    maskRed = mask1 | mask2

    # Create a green mask
    maskGreen = cv.inRange(hsv, lower_green, upper_green)
    #Bitwise-OR the masks
    mask = maskRed | maskGreen

    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)

    # Find contours
    contours_green, _ = cv.findContours(maskGreen, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv.findContours(maskRed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    if contours_green and contours_red:
        # Assume the largest contour for each color is the wall
        green_contour = max(contours_green, key=cv.contourArea)
        red_contour = max(contours_red, key=cv.contourArea)
        
        # Compute the centroids using image moments
        M_green = cv.moments(green_contour)
        M_red = cv.moments(red_contour)
        
        if M_green["m00"] > 0 and M_red["m00"] > 0:
            cx_green = int(M_green["m10"] / M_green["m00"])
            cx_red = int(M_red["m10"] / M_red["m00"])
            
            # Check if the green (left wall) is to the left of the red (right wall)
            if cx_green < cx_red:
                print("right direction")
            else:
                print("wrong direction")


    # Show the images
    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)

    # press q to close the window
    if cv.waitKey(5) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()