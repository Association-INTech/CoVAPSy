import numpy as np

from simulation import config as c


def camera_color_profile(image: np.ndarray) -> np.ndarray:
    """Return a 1-D red/green profile across the camera's horizontal view.

    For a full image, each output value represents one screen column and shows
    the balance of detected green versus red pixels in the lower half: positive
    means more green, negative means more red, and zero means neither or balanced.
    For a 1-D view shaped (3, width), each value classifies that position:
    +1 means green, -1 means red, and 0 means neither.
    """
    if c.input_type == "image":
        image = np.asarray(image, dtype=np.uint8).reshape(
            c.camera_vertical_resolution,
            c.camera_horizontal_resolution,
            3,
        )
        # Use the lower half of the full frame for the column profile.
        roi = image[image.shape[0] // 2 :, :, :]
        red_channel = roi[:, :, 0].astype(np.int16)
        green_channel = roi[:, :, 1].astype(np.int16)
        blue_channel = roi[:, :, 2].astype(np.int16)

        red = (
            (red_channel > 40)
            & (red_channel > green_channel * 1.3)
            & (red_channel > blue_channel * 1.3)
        )
        green = (
            (green_channel > 40)
            & (green_channel > red_channel * 1.3)
            & (green_channel > blue_channel * 1.3)
        )

        signed = green.astype(np.float32) - red.astype(np.float32)
        colored_count = (red | green).sum(axis=0)
        return signed.sum(axis=0) / np.maximum(colored_count, 1)

    else : 
        # The 1-D camera view has one row per color channel and one column per position.
        image = np.asarray(image, dtype=np.int16).reshape(
            3,
            c.camera_horizontal_resolution,
        )
        red_channel = image[0, :]
        green_channel = image[1, :]
        blue_channel = image[2, :]
    
        red = (
            (red_channel > 40)
            & (red_channel > green_channel * 1.3)
            & (red_channel > blue_channel * 1.3)
        )
        green = (
            (green_channel > 40)
            & (green_channel > red_channel * 1.3)
            & (green_channel > blue_channel * 1.3)
        )
    
        # There is one RGB sample per position, so each score is -1, 0, or +1.
        return green.astype(np.float32) - red.astype(np.float32)
    