import time
import numpy as np
from multiprocessing import Process
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.connection import Client
import logging
import os
from PIL import Image
import scipy as sp

from drivers.camera import run_camera


N_IMAGES = 100  # Number of images to capture
SAVE_DIR = "Captured_Frames"  # Directory to save frames
DEBUG_DIR = "Debug"  # Directory for debug images
DEBUG_DIR_wayfinding = "Debug_Wayfinding"  # Directory for wayfinding debug images
COLOUR_KEY = {"green": -1, "red": 1, "none": 0}
COLOR_THRESHOLD = 20  # Threshold for color intensity difference
Y_OFFSET = -40  # Offset for the y-axis in the image


class CameraProxy:
    def __init__(
        self,
        whep_url: str,
        w: int = 320,
        h: int = 240,
        rpc_addr=("127.0.0.1", 6000),
        authkey=b"covapsy",
    ):
        self.log = logging.getLogger(__name__)
        self.w = w
        self.h = h
        self.rpc_addr = rpc_addr
        self.authkey = authkey

        # the name of the shared memory block that the worker process will create and write the small BGR image to
        self.shm_name = f"covapsy_cam_small_{os.getpid()}"

        # start the worker process
        self._proc = Process(
            target=run_camera,
            args=(whep_url, self.shm_name, w, h, rpc_addr, authkey),
            daemon=True,
        )
        # cleanup any existing shared memory with the same name (in case it wasn't cleaned up properly last time)
        try:
            old_shm = SharedMemory(name=self.shm_name)
            old_shm.close()
            old_shm.unlink()
            self.log.warning(
                "CameraProxy: removed stale shared memory %s", self.shm_name
            )
        except FileNotFoundError:
            pass
        except Exception as e:
            self.log.warning("CameraProxy: could not cleanup stale SHM: %s", e)

        self._proc.start()

        # wait for the existity of the SHM
        t0 = time.time()
        while True:
            try:
                self._shm = SharedMemory(name=self.shm_name)
                break
            except FileNotFoundError:
                if time.time() - t0 > 5:
                    self.log.error(
                        "CameraProxy: shared memory not created after 5 seconds"
                    )
                    raise RuntimeError("CameraProxy: shared memory not created")
                time.sleep(0.05)

        self._buf = np.ndarray((h, w, 3), dtype=np.uint8, buffer=self._shm.buf)

        # connect RPC
        t0 = time.time()
        while True:
            try:
                self._rpc = Client(rpc_addr, authkey=authkey)
                break
            except ConnectionRefusedError:
                if time.time() - t0 > 5:
                    self.log.error(
                        "CameraProxy: RPC server not available after 5 seconds"
                    )
                    raise RuntimeError("CameraProxy: RPC not ready")
                time.sleep(0.05)

    def get_last_image(self) -> np.ndarray:
        # copy for thread safety (and to avoid issues if the worker process updates the buffer while we're using it)
        return self._buf.copy()

    def get_stats(self) -> dict:
        self._rpc.send({"cmd": "get_stats"})
        resp = self._rpc.recv()
        if not resp.get("ok"):
            raise RuntimeError(resp.get("error"))
        return resp["stats"]

    def stop(self) -> None:
        try:
            self._rpc.send({"cmd": "stop"})
            self._rpc.recv()
        except Exception:
            pass

        try:
            self._rpc.close()
        except Exception:
            pass

        try:
            self._shm.close()
        except Exception:
            pass

        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=2)

    def camera_matrix(self, vector_size=128, image=None) -> np.ndarray:
        """
        Create a matrix of -1, 0, and 1 for a line in the image. The matrix size is 128.
        """
        if image is None:
            image = self.get_last_image()
        height, width, _ = image.shape
        if vector_size > width:
            raise ValueError("Vector size cannot be greater than image width")

        # Slice the middle 5% of the image height
        sliced_image = image[
            height // 2 - height // 40 + Y_OFFSET : height // 2
            + height // 40
            + Y_OFFSET,
            :,
            :,
        ]

        # Ensure the width of the sliced image is divisible by vector_size
        adjusted_width = (width // vector_size) * vector_size
        sliced_image = sliced_image[:, :adjusted_width, :]

        # Initialize the output matrix
        output_matrix = np.zeros(vector_size, dtype=int)
        bucket_size = adjusted_width // vector_size

        # Calculate red and green intensities for all segments at once
        reshaped_red = sliced_image[:, :, 0].reshape(
            sliced_image.shape[0], vector_size, bucket_size
        )
        reshaped_green = sliced_image[:, :, 1].reshape(
            sliced_image.shape[0], vector_size, bucket_size
        )
        red_intensities = np.mean(reshaped_red, axis=(0, 2))
        green_intensities = np.mean(reshaped_green, axis=(0, 2))

        # Determine the color for each segment
        output_matrix[red_intensities > green_intensities + COLOR_THRESHOLD] = (
            COLOUR_KEY["red"]
        )
        output_matrix[green_intensities > red_intensities + COLOR_THRESHOLD] = (
            COLOUR_KEY["green"]
        )
        output_matrix[
            np.abs(red_intensities - green_intensities) <= COLOR_THRESHOLD
        ] = COLOUR_KEY["none"]

        # Recreate the image from the matrix
        if self.log.isEnabledFor(logging.DEBUG):
            path = os.path.join(
                DEBUG_DIR, f"debug_combined_image{self.debug_counter}.jpg"
            )
            self.recreate_image_from_matrix(
                sliced_image, output_matrix, adjusted_width, vector_size
            ).save(path)

        return output_matrix

    def recreate_image_from_matrix(
        self,
        image: np.ndarray,
        matrix: np.ndarray,
        adjusted_width: int,
        vector_size: int = 128,
    ) -> Image.Image:
        """
        Recreate an image from the matrix of -1, 0, and 1 and append it to the bottom of the sliced image.
        """

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
        self.debug_counter += 1
        return Image.fromarray(combined_image).convert("RGB")

    def is_green_or_red(self, lidar) -> int:
        """
        Check if the car is facing a green or red wall by analyzing the bottom half of the image.
        """
        image = self.get_last_image()
        height, _, _ = image.shape
        bottom_half = image[height // 2 :, :, :]  # Slice the bottom half of the image
        lidar = np.max(
            sp.ndimage.zoom(
                lidar[595:855], image.shape[1] / len(lidar[595:855]), mode="nearest"
            )[None, :],
            0,
        )  # Resize lidar data to match the image size
        print((lidar < 0.5).sum())
        print(f"min lidar: {lidar.min()}, max lidar: {lidar.max()}")
        red_intensity = np.mean(
            bottom_half[:, :, 0] * (lidar < 0.5)
        )  # Red channel in RGB
        green_intensity = np.mean(
            bottom_half[:, :, 1] * (lidar < 0.5)
        )  # Green channel in RGB

        if green_intensity > red_intensity + COLOR_THRESHOLD:
            return COLOUR_KEY["green"]
        elif red_intensity > green_intensity + COLOR_THRESHOLD:
            return COLOUR_KEY["red"]
        return COLOUR_KEY["none"]
