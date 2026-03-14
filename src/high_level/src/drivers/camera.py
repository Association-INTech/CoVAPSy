# this file is launched as a separate process by camera_proxy.py.
import asyncio
import logging
import threading
import time
from typing import Never, Optional, cast

import numpy as np
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.connection import Listener

import aiohttp
import cv2
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
    VideoStreamTrack,
)
from av import VideoFrame
from high_level.autotech_constant import (
    FREQUENCY_REVERSE_DETECTION,
    LIMIT_REVERSE_COUNT,
)

N_IMAGES = 100  # Number of images to capture
SAVE_DIR = "Captured_Frames"  # Directory to save frames
DEBUG_DIR = "Debug"  # Directory for debug images
DEBUG_DIR_wayfinding = "Debug_Wayfinding"  # Directory for wayfinding debug images
COLOUR_KEY = {"green": 1, "red": -1, "none": 0}
COLOR_THRESHOLD = 20  # Threshold for color intensity difference
Y_OFFSET = -80  # Offset for the y-axis in the image


class Camera_red_or_green:
    """
    Class representing the camera and its processing to determine if the car is facing a red or green wall.
    """

    def __init__(self, server) -> None:
        self.server = server
        self.log = logging.getLogger(__name__)
        self.is_reverse = False

        threading.Thread(target=self.thread_check_reverse, daemon=True).start()

    def is_running_in_reversed(
        self, image: Optional[np.ndarray] = None, LEFT_IS_GREEN: bool = True
    ) -> bool:
        """
        Check if the car is running in reverse.
        If the car is in reverse, green will be on the right side of the image and red on the left.
        """
        if image is None:
            image = self.server.camera.get_last_image()
        matrix = self.server.camera.camera_matrix(image=image)
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
            """
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
                debug_slice_image = self.server.camera.recreate_image_from_matrix(
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
                )"""
            return True
        elif not LEFT_IS_GREEN and average_green_index > average_red_index:
            return True
        return False

    def thread_check_reverse(self) -> Never:
        """
        Thread function to continuously check if the car is running in reverse and update the server state.
        """
        temp = 0
        while self.server.camera is None:
            time.sleep(1)  # Wait until the camera is initialized
        while True:
            try:
                image = self.server.camera.get_last_image()

                result = self.is_running_in_reversed(image=image)

                if result:
                    temp += 1
                else:
                    temp = 0

                if temp >= LIMIT_REVERSE_COUNT and not self.is_reverse:
                    self.log.info("Car is running in reverse")
                    self.is_reverse = True
                elif temp == 0 and self.is_reverse:
                    self.log.info("Car is no longer running in reverse")
                    self.is_reverse = False
            except Exception as e:
                self.log.error(f"Error checking reverse: {e}")
            time.sleep(FREQUENCY_REVERSE_DETECTION)


class Camera:
    """
    Camera = client WebRTC (WHEP) vers MediaMTX.
    MediaMTX ouvre la PiCam (source: rpiCamera). Python ne fait que consommer.
    """

    def __init__(self, whep_url: str = "http://192.168.0.20:8889/cam/whep") -> None:
        self.log = logging.getLogger(__name__)
        self.log.info("Initializing Camera with WHEP URL: %s", whep_url)
        self.whep_url = whep_url
        self.debug_counter = 0
        self.last_frame = np.zeros((0, 0, 0))
        self._lock = threading.Lock()

        self._stop_flag = threading.Event()
        self._frame_queue: asyncio.Queue[VideoFrame] = asyncio.Queue(maxsize=2)
        self._thread = threading.Thread(target=self._thread_main, daemon=True)

        self._thread.start()

    # --------- thread -> event loop asyncio ---------
    def _thread_main(self) -> None:
        asyncio.run(self._run_forever())

    async def _run_forever(self) -> None:
        # boucle de reconnexion automatique
        while not self._stop_flag.is_set():
            try:
                await self._run_once(self.whep_url)
            except Exception as e:
                self.log.warning("WHEP client error: %s", e)

            # petit backoff avant de retenter
            await asyncio.sleep(0.5)

    async def _processing_loop(self) -> None:
        while not self._stop_flag.is_set():
            frame = await self._frame_queue.get()
            # H264 decoding -> numpy (OUTSIDE WebRTC)
            img_bgr = frame.to_ndarray(format="bgr24")
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            # optional but recommended (algorithm stability)
            img_rgb = cv2.resize(img_rgb, (320, 240))

            with self._lock:
                self.last_frame = img_rgb

    async def _wait_ice_complete(
        self, pc: RTCPeerConnection, timeout: float = 2.0
    ) -> None:
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

    async def _run_once(self, url: str) -> None:
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
        async def on_track(track: VideoStreamTrack) -> None:
            if track.kind != "video":
                return

            self.log.info("WHEP: receiving video track")
            frame_received.set()

            processing_task = asyncio.create_task(self._processing_loop())

            try:
                while not self._stop_flag.is_set():
                    frame = cast(VideoFrame, await track.recv())

                    # NEVER block here
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

        # wait until we have a video track (otherwise useless)
        try:
            await asyncio.wait_for(frame_received.wait(), timeout=5)
        except asyncio.TimeoutError:
            await pc.close()
            raise RuntimeError("WHEP: no video track received")

        # stay alive as long as not stopped
        while not self._stop_flag.is_set() and pc.connectionState != "closed":
            await asyncio.sleep(0.1)

        await pc.close()

    # ----------------------------
    # Program interface
    # ----------------------------

    def stop(self) -> None:
        self.log.info("Camera: stopping")
        self._stop_flag.set()
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=2)
        except Exception as e:
            self.log.warning("Camera: thread join failed: %s", e)
        self.log.info("Camera: stopped")

    # ----------
    # Public API
    # ----------
    def get_last_image(self) -> np.ndarray:
        with self._lock:
            return self.last_frame.copy()


def run_camera(
    whep_url: str,
    shm_name: str,
    w: int,
    h: int,
    rpc_addr=("127.0.0.1", 6000),
    authkey=b"covapsy",
) -> None:

    log = logging.getLogger(__name__)
    try:
        shm = SharedMemory(name=shm_name, create=True, size=w * h * 3)
    except FileExistsError:
        old = SharedMemory(name=shm_name)
        old.close()
        old.unlink()
        shm = SharedMemory(name=shm_name, create=True, size=w * h * 3)

    buf = np.ndarray((h, w, 3), dtype=np.uint8, buffer=shm.buf)
    buf[:] = 0

    cam = Camera(whep_url)
    stop_flag = threading.Event()

    def writer_loop():
        while not stop_flag.is_set():
            frame = cam.get_last_image()
            if frame is not None and frame.shape == buf.shape:
                buf[:] = frame
            time.sleep(0.01)

    t = threading.Thread(target=writer_loop, daemon=True)
    t.start()

    listener = Listener(rpc_addr, authkey=authkey)

    try:
        conn = listener.accept()  # un seul client suffit
        while not stop_flag.is_set():
            try:
                msg = conn.recv()
            except EOFError:
                log.info("run_camera: RPC connection closed by peer")
                break
            except KeyboardInterrupt:
                log.info("run_camera: KeyboardInterrupt received")
                break
            except OSError as e:
                log.info("run_camera: RPC receive stopped: %s", e)
                break

            cmd = msg.get("cmd")
            if cmd == "ping":
                conn.send({"ok": True, "msg": "pong"})
            elif cmd == "stop":
                conn.send({"ok": True})
                stop_flag.set()
            else:
                conn.send({"ok": False, "error": f"unknown cmd {cmd}"})
    finally:
        stop_flag.set()
        try:
            cam.stop()
        except Exception:
            pass
        try:
            listener.close()
        except Exception:
            pass
        shm.close()
        shm.unlink()

    def stop(self) -> None:
        self.log.info("CameraProxy: stopping")

        # 1) Demander l'arrêt au worker
        if self._rpc is not None:
            try:
                self._rpc.send({"cmd": "stop"})
                self._rpc.recv()
            except (BrokenPipeError, EOFError, OSError) as e:
                self.log.warning("CameraProxy: RPC stop failed: %s", e)
            except Exception as e:
                self.log.warning("CameraProxy: unexpected RPC stop error: %s", e)

        # 2) Attendre que le process finisse
        if self._proc is not None:
            try:
                self._proc.join(timeout=3)
                if self._proc.is_alive():
                    self.log.warning("CameraProxy: worker still alive, terminating")
                    self._proc.terminate()
                    self._proc.join(timeout=1)
            except Exception as e:
                self.log.warning("CameraProxy: process join/terminate failed: %s", e)

        # 3) Fermer la SHM côté proxy
        if self._shm is not None:
            try:
                self._shm.close()
            except Exception as e:
                self.log.warning("CameraProxy: shm.close failed: %s", e)

        # 4) Tentative de cleanup final
        try:
            shm = SharedMemory(name=self.shm_name)
            shm.close()
            shm.unlink()
        except FileNotFoundError:
            pass
        except Exception as e:
            self.log.warning("CameraProxy: shm unlink cleanup failed: %s", e)

        self.log.info("CameraProxy: stopped")
