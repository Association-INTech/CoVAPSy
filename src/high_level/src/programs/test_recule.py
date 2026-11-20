from programs import Program
import numpy as np
import logging
import threading
import time

from high_level.autotech_constant import LIDAR_RESOLUTION, FOV_DEG
from programs.car import Crashcar

class Test_recule(Program):
    def __init__(self, server):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.server = server
        self.controls_car = True
        self.speed = 0
        self.dir = 0
        self.running = False
        self.crash_car = None

    @property
    def target_speed(self) -> float:
        return 0  # toujours immobile pour ce test

    @property
    def direction(self) -> float:
        return 0

    @property
    def lidar(self):
        return self.server.lidar

    def main(self) -> None:
        # attendre que le lidar soit prêt
        while self.running and self.lidar is None:
            self.log.debug("Lidar not ready")
            time.sleep(0.1)

        self.crash_car = Crashcar(LIDAR_RESOLUTION, np.deg2rad(FOV_DEG), self.server)
        self.log.info("TEST CRASH: voiture immobile, lecture des ratios")

        while self.running:
            try:
                lidar_m = np.asarray(self.lidar.r_distance, dtype=np.float32) / 1000
                crashed, zone = self.crash_car.detect(lidar_m)
                self.log.info(
                    f"Ratios {np.round(self.crash_car.last_ratios, 2)} | "
                    f"crashed={crashed} zone={zone}"
                )
            except Exception as e:
                self.log.error(f"Crash test error: {e}")
            time.sleep(0.2)

    def start(self):
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.main, daemon=True).start()

    def kill(self):
        self.running = False