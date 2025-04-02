import numpy as np
from PIL import Image

COLOUR_KEY = {
    "green": 1,
    "red": -1,
    "none": 0
}
COLOR_THRESHOLD = 20  # Threshold for color intensity difference
Y_OFFSET = 0.5  # Offset for the y-axis in the image
counter = 1


def get_intensity(image, color):
    """Calculate the average intensity of a specific color channel in the image."""
    return np.mean(image[:, :, color])

def is_running_in_reversed(image, LEFT_IS_GREEN=True):
    """
    Check if the car is running in reverse.
    If the car is in reverse, green will be on the right side of the image and red on the left.
    """
    height, width, _ = image.shape
    left_half = image[:, :width // 2]
    right_half = image[:, width // 2:]

    left_red_intensity = np.mean(left_half[:, :, 0])  # Red channel in RGB
    right_red_intensity = np.mean(right_half[:, :, 0])  # Red channel in RGB
    
    # Allow to change the color of the left and right side
    return (left_red_intensity <= right_red_intensity) if LEFT_IS_GREEN else (left_red_intensity > right_red_intensity)

def is_green_or_red(image):
    """
    Check if the car is facing a green or red wall by analyzing the bottom half of the image.
    """
    height, _, _ = image.shape
    bottom_half = image[height // 2:, :, :]  # Slice the bottom half of the image

    red_intensity = np.mean(bottom_half[:, :, 0])  # Red channel in RGB
    green_intensity = np.mean(bottom_half[:, :, 1])  # Green channel in RGB

    if green_intensity > red_intensity + COLOR_THRESHOLD:
        return COLOUR_KEY["green"]
    elif red_intensity > green_intensity + COLOR_THRESHOLD:
        return COLOUR_KEY["red"]
    return COLOUR_KEY["none"]

    
def recreate_image_from_matrix(image, matrix,adjusted_width, vector_size=128):
    """
    Recreate an image from the matrix of -1, 0, and 1 and append it to the bottom of the sliced image.
    """
    global counter

    # Create a blank image (20 pixels high)
    recreated_image = np.zeros((20, vector_size, 3), dtype=np.uint8)
    recreated_image[:, matrix == COLOUR_KEY["red"], :] = [255, 0, 0]  # Red
    recreated_image[:, matrix == COLOUR_KEY["green"], :] = [0, 255, 0]  # Green
    recreated_image[:, matrix == COLOUR_KEY["none"], :] = [128, 128, 128]  # Gray

    # Resize the recreated image to match the width of the sliced image
    scale_factor = adjusted_width // vector_size
    recreated_image_resized = np.repeat(recreated_image, scale_factor, axis=1)

    # Adjust the width of the recreated image to match the sliced image
    if recreated_image_resized.shape[1] > adjusted_width:
        recreated_image_resized = recreated_image_resized[:, :adjusted_width, :]
    elif recreated_image_resized.shape[1] < adjusted_width:
        padding = adjusted_width - recreated_image_resized.shape[1]
        recreated_image_resized = np.pad(
            recreated_image_resized,
            ((0, 0), (0, padding), (0, 0)),
            mode="constant",
            constant_values=0,
        )
        recreated_image_resized[:, -padding:, 2] = 255  # Blue channel for padding

    # Append the recreated image to the bottom of the sliced image
    combined_image = np.vstack((image, recreated_image_resized))
    Image.fromarray(combined_image).convert("RGB").save(f"debug_combined_image{counter}.jpg")
    counter += 1

def camera_matrix(image, vector_size=128):
    """
    Create a matrix of -1, 0, and 1 for a line in the image. The matrix size is 128.
    """
    height, width, _ = image.shape
    if vector_size > width:
        raise ValueError("Vector size cannot be greater than image width")

    # Slice the middle 5% of the image height
    sliced_image = image[height // 2 - height // 40 : height // 2 + height // 40, :, :]

    # Ensure the width of the sliced image is divisible by vector_size
    adjusted_width = (width // vector_size) * vector_size
    sliced_image = sliced_image[:, :adjusted_width, :]

    # Initialize the output matrix
    output_matrix = np.zeros(vector_size, dtype=int)
    bucket_size = adjusted_width // vector_size

    # Calculate red and green intensities for all segments at once
    reshaped_red = sliced_image[:, :, 0].reshape(sliced_image.shape[0], vector_size, bucket_size)
    reshaped_green = sliced_image[:, :, 1].reshape(sliced_image.shape[0], vector_size, bucket_size)
    red_intensities = np.mean(reshaped_red, axis=(0, 2))
    green_intensities = np.mean(reshaped_green, axis=(0, 2))

    # Determine the color for each segment
    output_matrix[red_intensities > green_intensities + COLOR_THRESHOLD] = COLOUR_KEY["red"]
    output_matrix[green_intensities > red_intensities + COLOR_THRESHOLD] = COLOUR_KEY["green"]
    output_matrix[np.abs(red_intensities - green_intensities) <= COLOR_THRESHOLD] = COLOUR_KEY["none"]

    # Recreate the image from the matrix
    recreate_image_from_matrix(sliced_image, output_matrix, adjusted_width, vector_size)

    return output_matrix

if __name__ == "__main__":
    # Example usage:
    # open image
    images_paths = [
        "src\HL\Vision\Image_Piste\image.jpg",
        "src\HL\Vision\Image_Piste\image2.jpg",
        "src\HL\Vision\Image_Piste\image3.jpg",
        "src\HL\Vision\Image_Piste\image4.png",
        "src\HL\Vision\Image_Piste\image5.png",
        "src\HL\Vision\Image_Piste\image6.png",
        "src\HL\Vision\Image_Piste\image7.png",
        "src\HL\Vision\Image_Piste\image8.png",
        "src\HL\Vision\Image_Piste\image9.png",
        "src\HL\Vision\Image_Piste\image10.png",
        "src\HL\Vision\Image_Piste\image11.png",
        "src\HL\Vision\Image_Piste\image12.png",
        "src\HL\Vision\Image_Piste\image13.png"
    ]
    for image_path in images_paths:
        # print(f"Processing image: {image_path}")
        pil_image = Image.open(image_path).convert("RGB")  # Open and ensure it's in RGB format
        image = np.array(pil_image)  # Convert to NumPy array
        is_green_or_red_bool= is_green_or_red(image)
        is_running_in_reverse = is_running_in_reversed(image)
        camera_matrix(image)
        print(f"Image: {image_path}, is_green_or_red: {is_green_or_red_bool}, is_running_in_reverse: {is_running_in_reverse}")
        
        