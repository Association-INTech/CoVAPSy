import logging
from enum import Enum, auto
from typing import Dict, Tuple, cast

import numpy as np
from controller import Camera, Lidar, TouchSensor
from vehicle import Driver

from simulation import config as c
from utils.simulation_rank import get_simulation_rank


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


class State(Enum):
    AI = auto()
    BACK = auto()


class VehicleDriver(Driver):
    """
    This class is a subclass of the Driver class and is used to control the vehicle.
    It basically receives instructions from the controller_world_supervisor and follows them.
    """

    def __init__(self):
        super().__init__()
        self.state = State.AI

        basicTimeStep = int(self.getBasicTimeStep())
        self.sensorTime = basicTimeStep // 4

        self.v_min = c.v_min
        self.v_max = c.v_max

        self.applied_steering = 0.0
        self.applied_speed = 0.0

        self.vehicle_rank = int(self.getName().split("_")[-1])
        # Lidar
        self.lidar = cast(Lidar, self.getDevice("Hokuyo"))
        self.lidar.enable(self.sensorTime)
        self.lidar.enablePointCloud()

        # Camera
        self.camera = cast(Camera, self.getDevice("RASPI_Camera_V2"))
        self.camera.enable(self.sensorTime)

        # Checkpoint sensor
        self.touch_sensor = cast(TouchSensor, self.getDevice("touch_sensor"))
        self.touch_sensor.enable(self.sensorTime)

        self.simulation_rank = get_simulation_rank()

        # Logger
        self.handler = logging.FileHandler(
            f"/tmp/autotech/vehicle_{self.simulation_rank}_{self.vehicle_rank}.log"
        )
        self.handler.setFormatter(c.FORMATTER)
        self.log = logging.getLogger(
            f"VEHICLE_{self.simulation_rank}_{self.vehicle_rank}"
        )
        self.log.setLevel(level=c.LOG_LEVEL)
        self.log.addHandler(self.handler)

        self.log.debug("Connection to the server")
        self.fifo_r = open(
            f"/tmp/autotech/serverto{self.simulation_rank}_{self.vehicle_rank}.pipe",
            "rb",
        )
        self.log.debug("Connection with the supervisor")
        self.fifo_w = open(
            f"/tmp/autotech/{self.simulation_rank}_{self.vehicle_rank}tosupervisor.pipe",
            "wb",
        )

    # Check the state of the car
    def observe(self) -> Dict[str, np.ndarray]:
        # try:
        sensor_data = np.array([self.touch_sensor.getValue()], dtype=np.float32)
        lidar_data = np.array(self.lidar.getRangeImage(), dtype=np.float32)

        if c.camera_horizontal_resolution == 0:
            # empty array
            camera_data = np.zeros([0], dtype=np.float32)
        else:
            camera_data = np.array(self.camera.getImageArray(), dtype=np.float32)
            # shape = (camera_horizontal_resolution, 1, 3)
            camera_data = camera_data.transpose(1, 2, 0)[0]
            # shape = (3, 1080)
            color = np.argmax(camera_data, axis=0)
            camera_data = (
                (color == 0).astype(np.float32) * -1
                + (color == 1).astype(np.float32) * 1
                + (color == 2).astype(np.float32) * 0
            )
            # red   -> -1
            # green -> 1
            # blue  -> 0

        return {
            "Collision": sensor_data,
            "Lidar": lidar_data,
            "Cam": camera_data,
        }

        # except:
        #     # In case of no lidar return
        #     return (
        #         np.array(self.touch_sensor.getValue(), dtype=np.float32),
        #         np.zeros(self.lidar.getNumberOfPoints(), dtype=np.float32),
        #     )

    # Step function of the GYM environment
    def step(self):
        # we cannot obs until an action has been applied,
        # otherwise, supervisor would have computed the reward for the previous action
        # then would have forwarded it to env that would compute the next action
        # so the reward would be for the previous action, not the current one
        # self.log.debug("Waiting for action from SERVER_{self.simulation_rank}_{self.vehicle_rank}")
        action = np.frombuffer(
            self.fifo_r.read(np.dtype(np.int64).itemsize * 2), dtype=np.int64,
        )
        # self.log.info(f"received {action=}")

        action_steering = np.linspace(
            -0.4, 0.4, c.n_actions_steering, dtype=np.float32
        )[action[0]]
        
        action_speed = np.linspace(
            c.v_min, c.v_max, c.n_actions_speed, dtype=np.float32
            )[action[1]]
        
        cur_angle = self.getSteeringAngle()
        dt = self.getBasicTimeStep() / 1000.0
        omega = 20 # max angular speed of the steering wheel in rad/s

        action_steering = cur_angle + np.clip(action_steering - cur_angle, -omega * dt, omega * dt)

        self.setSteeringAngle(float(action_steering))
        self.setCruisingSpeed(float(action_speed))
        self.applied_steering = float(action_steering)
        self.applied_speed = float(action_speed)

        if self.state == State.BACK:
            self.back()
        
        if super().step() == -1:
            return -1
        
        if self.touch_sensor.getValue() > 0:
            self.state = State.BACK
        
        obs = self.observe()
        # self.log.info(f"observed {obs=}")
        self._send_observation(obs)
        return 0

    def _send_observation(self, obs):
        # sends lidar, camera, sensor, cruising speed, steering angle to the supervisor
        state = np.concatenate(
            [
                obs["Collision"],
                obs["Lidar"],
                obs["Cam"],
                np.array([self.applied_steering], dtype=np.float32),
                np.array([self.applied_speed], dtype=np.float32),
            ]
        )
        self.fifo_w.write(state.tobytes())
        self.fifo_w.flush()

    def back(self):
        # if wall on "dir": turn to "dir" and reverse until able to move forward (wall distance to verify)
        lidar, cam = self.observe()["Lidar"], self.observe()["Cam"]

        S = sum(cam)
        dir = S > 0
        if dir:
            self.setSteeringAngle(0.33)
            if too_close(lidar, dir):
                self.setCruisingSpeed(-2)
            else:
                self.state = State.AI
        else:
            self.setSteeringAngle(-0.33)
            if too_close(lidar, dir):
                self.setCruisingSpeed(-2)
            else:
                self.state = State.AI

    def run(self):
        # this call is just there to make sure at least one step
        # is done in the entire simulation before we call lidar.getRangeImage()
        # otherwise it will crash the controller with the message:
        # WARNING: 'controller_vehicle_driver' controller crashed.
        # WARNING: controller_vehicle_driver: The process crashed some time after starting successfully.
        if super().step() == -1:
            return -1
        self.applied_steering = float(self.getSteeringAngle())
        initial_obs = self.observe()
        # self.log.info(f"observed {initial_obs=}")
        self._send_observation(initial_obs)
        
        while self.step() != -1:
            pass


# ----------------Main Program--------------------
def main():
    driver = VehicleDriver()
    driver.log.info("Starting the vehicle driver")
    driver.run()


if __name__ == "__main__":
    main()
