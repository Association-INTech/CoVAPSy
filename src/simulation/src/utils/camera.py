import numpy as np

from simulation import config as c


def camera_color_profile(image: np.ndarray) -> np.ndarray:
    """Return one signed red/green score per image column."""
    image = np.asarray(image, dtype=np.uint8).reshape(
        c.camera_vertical_resolution,
        c.camera_horizontal_resolution,
        3,
    )
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
