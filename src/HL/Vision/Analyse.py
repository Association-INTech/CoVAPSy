import numpy as np


def is_reversed(image):
    """ Check if the car is running in reverse. If the car is in reverse, green will be on the left side of the image and red on the right side. """
    height, width, _ = image.shape
    left_half = image[:, :width // 2]
    right_half = image[:, width // 2:]

    # Calculate the average red intensity for each half
    left_red_intensity = np.mean(left_half[:, :, 0])  # Red channel in RGB
    right_red_intensity = np.mean(right_half[:, :, 0])  # Red channel in RGB

    # Determine the side with the higher red intensity
    if left_red_intensity > right_red_intensity:
        return "left"
    elif right_red_intensity > left_red_intensity:
        return "right"
    else:
        return "none"

def is_green_or_red(image):
    """ Check if the car is facing a green or red wall"""
    pass

def camera_matrix(image):
    """ Create a matrix of -1, 0 and 1 for a line in the image. The matrix size is of """
    pass

if __name__ == "__main__":
    # Example usage:
    # open image
    result = is_reversed(image)
    print(f"The car is running in: {result}")