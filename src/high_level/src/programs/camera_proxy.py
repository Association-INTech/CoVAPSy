import time
import numpy as np
from multiprocessing import Process
from multiprocessing.shared_memory import SharedMemory
from multiprocessing.connection import Client

from drivers.camera import run_camera

COLOUR_KEY = {"green": 1, "red": -1, "none": 0}


class CameraProxy:
    def __init__(
        self,
        whep_url: str,
        w: int = 320,
        h: int = 240,
        rpc_addr=("127.0.0.1", 6000),
        authkey=b"covapsy",
    ):
        self.w = w
        self.h = h
        self.rpc_addr = rpc_addr
        self.authkey = authkey

        # the nale of the shared memory block that the worker process will create and write the small BGR image to
        self.shm_name = "covapsy_cam_small"

        # start the worker process
        self._proc = Process(
            target=run_camera,
            args=(whep_url, self.shm_name, w, h, rpc_addr, authkey),
            daemon=True,
        )
        self._proc.start()

        # wait for the existity of the SHM
        t0 = time.time()
        while True:
            try:
                self._shm = SharedMemory(name=self.shm_name)
                break
            except FileNotFoundError:
                if time.time() - t0 > 5:
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
