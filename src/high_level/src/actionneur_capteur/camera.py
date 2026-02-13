import asyncio
import logging
import os
import threading
import time
from typing import Optional, cast

import aiohttp
import cv2
import numpy as np
import scipy as sp
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from av import VideoFrame
from PIL import Image

N_IMAGES = 100  # Number of images to capture
SAVE_DIR = "Captured_Frames"  # Directory to save frames
DEBUG_DIR = "Debug"  # Directory for debug images
DEBUG_DIR_wayfinding = "Debug_Wayfinding"  # Directory for wayfinding debug images
COLOUR_KEY = {"green": 1, "red": -1, "none": 0}
COLOR_THRESHOLD = 20  # Threshold for color intensity difference
Y_OFFSET = -80  # Offset for the y-axis in the image


class Camera:
    """
    Camera = client WebRTC (WHEP) vers MediaMTX.
    MediaMTX ouvre la PiCam (source: rpiCamera). Python ne fait que consommer.
    """

    def __init__(self, whep_url: str = "http://192.168.0.20:8889/cam/whep"):
        self.log = logging.getLogger(__name__)
        self.whep_url = whep_url
        self.debug_counter = 0
        self.last_frame = np.zeros((0, 0, 0))
        self._lock = threading.Lock()

        self._stop_flag = threading.Event()
        self._frame_queue: asyncio.Queue[VideoFrame] = asyncio.Queue(maxsize=2)
        self._thread = threading.Thread(target=self._thread_main, daemon=True)

        self._thread.start()

    # --------- thread -> event loop asyncio ---------
    def _thread_main(self):
        asyncio.run(self._run_forever())

    async def _run_forever(self):
        # boucle de reconnexion automatique
        while not self._stop_flag.is_set():
            try:
                await self._run_once(self.whep_url)
            except Exception as e:
                self.log.warning("WHEP client error: %s", e)

            # petit backoff avant de retenter
            await asyncio.sleep(0.5)

    async def _processing_loop(self):
        while not self._stop_flag.is_set():
            frame = await self._frame_queue.get()
            # Décodage H264 -> numpy (HORS WebRTC)
            img_bgr = frame.to_ndarray(format="bgr24")
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            # optionnel mais recommandé (stabilité algo)
            img_rgb = cv2.resize(img_rgb, (320, 240))

            with self._lock:
                self.last_frame = img_rgb

    async def _wait_ice_complete(self, pc: RTCPeerConnection, timeout: float = 2.0):
        if pc.iceGatheringState == "complete":
            return

        ev = asyncio.Event()

        @pc.on("icegatheringstatechange")
        def _on_ice():
            if pc.iceGatheringState == "complete":
                ev.set()

        try:
            await asyncio.wait_for(ev.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            self.log.warning("ICE gathering not complete after timeout")
            pass

    async def _run_once(self, url: str):
        config = RTCConfiguration(iceServers=[])

        pc = RTCPeerConnection(configuration=config)
        pc.addTransceiver("video", direction="recvonly")

        frame_received = asyncio.Event()

        @pc.on("connectionstatechange")
        async def on_state():
            if pc.connectionState in ("failed", "disconnected", "closed"):
                self.log.warning("WebRTC state: %s", pc.connectionState)
                await pc.close()

        @pc.on("track")
        async def on_track(track: VideoStreamTrack):
            if track.kind != "video":
                return

            self.log.info("WHEP: receiving video track")
            frame_received.set()

            processing_task = asyncio.create_task(self._processing_loop())

            try:
                while not self._stop_flag.is_set():
                    frame = cast(VideoFrame, await track.recv())

                    # ne JAMAIS bloquer ici
                    if self._frame_queue.full():
                        try:
                            self._frame_queue.get_nowait()
                        except asyncio.QueueEmpty:
                            pass

                    await self._frame_queue.put(frame)

            except Exception as e:
                self.log.warning("WebRTC track ended: %s", e)
            processing_task.cancel()

        # --- offer/answer WHEP ---
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        await self._wait_ice_complete(pc, timeout=3.0)

        async with aiohttp.ClientSession() as session:
            async with session.post(
                url,
                data=pc.localDescription.sdp,
                headers={"Content-Type": "application/sdp"},
                timeout=aiohttp.ClientTimeout(total=5),
            ) as resp:
                if resp.status != 201:
                    raise RuntimeError(f"WHEP failed: HTTP {resp.status}")
                answer_sdp = await resp.text()

        await pc.setRemoteDescription(
            RTCSessionDescription(sdp=answer_sdp, type="answer")
        )

        # attendre qu’on ait bien une piste vidéo (sinon inutile)
        try:
            await asyncio.wait_for(frame_received.wait(), timeout=5)
        except asyncio.TimeoutError:
            await pc.close()
            raise RuntimeError("WHEP: no video track received")

        # rester vivant tant que pas stoppé
        while not self._stop_flag.is_set() and pc.connectionState != "closed":
            await asyncio.sleep(0.1)

        await pc.close()

    # --------- API publique ---------
    def stop(self):
        """Arrête le client WHEP (ne stoppe pas MediaMTX)."""
        self._stop_flag.set()

    def get_last_image(self) -> np.ndarray:
        with self._lock:
            return self.last_frame.copy()

    # ----------------------------------------------------------
    # Interface publique
    # ----------------------------------------------------------

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

    def is_green_or_red(self, lidar):
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

    def is_running_in_reversed(
        self, image: Optional[np.ndarray] = None, LEFT_IS_GREEN: bool = True
    ):
        """
        Check if the car is running in reverse.
        If the car is in reverse, green will be on the right side of the image and red on the left.
        """
        if image is None:
            image = self.get_last_image()
        matrix = self.camera_matrix(image=image)
        if COLOUR_KEY["green"] not in matrix or COLOUR_KEY["red"] not in matrix:
            # If there are no green or no red pixels, return False
            return False
        green_indices = (matrix == COLOUR_KEY["green"]) * np.arange(1, len(matrix) + 1)
        average_green_index = np.mean(
            green_indices[green_indices > 0]
        )  # Average index of green

        red_indices = (matrix == COLOUR_KEY["red"]) * np.arange(1, len(matrix) + 1)
        average_red_index = np.mean(
            red_indices[red_indices > 0]
        )  # Average index of redcolor is red

        if LEFT_IS_GREEN and average_red_index > average_green_index:
            if self.log.isEnabledFor(logging.DEBUG):
                self.log.debug(
                    f"green: {average_green_index}, red: {average_red_index}"
                )
                vector_size = 128
                self.debug_counter += 1
                height, width, _ = image.shape
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
                debug_slice_image = self.recreate_image_from_matrix(
                    sliced_image, matrix, adjusted_width, vector_size
                )

                debug_slice_image.save(
                    os.path.join(
                        DEBUG_DIR_wayfinding,
                        f"wrong_direction_{self.debug_counter}_slice.jpg",
                    )
                )
                Image.fromarray(image).convert("RGB").save(
                    os.path.join(
                        DEBUG_DIR_wayfinding, f"wrong_direction{self.debug_counter}.jpg"
                    )
                )
            return True
        elif not LEFT_IS_GREEN and average_green_index > average_red_index:
            return True


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    cam = Camera("http://192.168.1.10:8889/cam/whep")

    print("Got frame. camera_matrix:", cam.camera_matrix()[:10])
    time.sleep(5)
    print("seconde frame", cam.camera_matrix()[:10])
    print("Ctrl+C to exit")
    try:
        while True:
            print(cam.camera_matrix()[:10])
            time.sleep(0.1)
    except KeyboardInterrupt:
        cam.stop()
