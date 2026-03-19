from programs import Program
import numpy as np
import logging
import threading


def too_close(lidar, dir):
    R = 0.83
    length = len(lidar)
    straight = lidar[length // 2]
    if dir:
        nearest = min(lidar[length // 2 :])
    else:
        nearest = min(lidar[: length // 2])

    cos = nearest / straight

    # I don't know why sometimes this happens
    if cos < -1 or cos > 1:
        return True

    theta = np.arccos(cos)
    L = R * (1 - np.sin(theta))
    return nearest < L


class Test_recule(Program):
    def __init__(self, server):
        super().__init__()
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

    def back(self):
        # if wall on "dir": turn to "dir" and reverse until able to move forward (wall distance to verify)
        lidar, cam = self.lidar.r_distance, self.camera.camera_matrix
        S = sum(cam)
        dir = S > 0
        if dir:
            self.dir = 18
            if too_close(lidar, dir):
                self.speed = -2
            else:
                self.state = 0
        else:
            self.dir = -18
            if too_close(lidar, dir):
                self.speed = -2
            else:
                self.state = 0

    def main(self) -> None:
        while self.running:
            if self.server.crash_car.crashed:
                self.state = 1

            if self.state == 1:
                self.back()
            else:
                self.dir = 0
                self.speed = 0

    def start(self):
        self.running = False
        threading.Thread(target=self.main, daemon=True).start()

    def kill(self):
        self.running = True
        self.speed = 0
