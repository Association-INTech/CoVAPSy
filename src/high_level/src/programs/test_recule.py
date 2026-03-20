from programs import Program
import numpy as np
import logging
import threading
import time


class Test_recule(Program):
    def __init__(self, server):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.server = server
        self.controls_car = True
        self.speed = 0
        self.dir = 0
        self.state = 0
        self.running = False

    @property
    def target_speed(self) -> float:
        return self.speed

    @property
    def direction(self):
        return self.dir

    @property
    def lidar(self):
        return self.server.lidar

    @property
    def camera(self):
        return self.server.camera

    def too_close(self, lidar, dir):
        R = 0.83
        length = len(lidar)
        straight = lidar[length // 2]
        lidar2 = [10000 if p == 0 else p for p in lidar]
        if dir:
            nearest = min(lidar2[length // 2 : 300])
        else:
            nearest = min(lidar2[300 : length // 2])

        cos = nearest / straight

        # I don't know why sometimes this happens
        if cos < -1 or cos > 1:
            return True

        theta = np.arccos(cos)
        L = R * (1 - np.sin(theta))
        self.log.error(f"L:{L}")
        return nearest < L * 1000

    def back(self):
        # if wall on "dir": turn to "dir" and reverse until able to move forward (wall distance to verify)
        lidar, cam = self.lidar.r_distance, self.camera.camera_matrix()
        S = sum(cam)
        dir = S > 0
        if dir:
            self.dir = 18
            if self.too_close(lidar, dir):
                self.speed = -6
            else:
                self.state = 0
                self.log.info("crashed finish")
        else:
            self.dir = -18
            if self.too_close(lidar, dir):
                self.speed = -6
            else:
                self.state = 0
                self.log.info("crashed finish")

    def main(self) -> None:
        while self.running:
            if self.server.crash_car.crashed:
                self.log.info("crash_car.crashed")
                self.state = 1

            if self.state == 1:
                self.back()
            else:
                self.dir = 0
                self.speed = 0
            time.sleep(0.01)

    def start(self):
        self.running = True
        threading.Thread(target=self.main, daemon=True).start()

    def kill(self):
        self.running = True
        self.speed = 0
