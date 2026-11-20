import logging
from enum import Enum, auto
from typing import Dict, Tuple, cast

import numpy as np
from controller import Camera, Lidar, TouchSensor
from vehicle import Driver

from simulation import config as c
from utils.fifo import FifoClosedError, read_exact
from utils.simulation_rank import get_simulation_rank

FOV_DEG = 270
WIDTH_LIDAR_CAR = 0.2 # TODO : mesurer
LENGTH_LIDAR_CAR = 0.2 # TODO : mesurer
ANGLE_TOLERANCE_CRASH = 5 # TODO : mesurer (en degre)

CALIB_STEPS = 150 
WRONG_STEPS = 50      
UTURN_OK_STEPS = 30   
SCORE_MIN = 0.05

V_MIN = 1
V_MAX = 5
V_BACK = -2
V_UTURN = 2


class State(Enum):
    AI = auto()
    BACK = auto()
    UTURN = auto()


def track_score(cam):
    """ > 0 : green rather on the right and red on the left
        < 0 : the opposite."""
    if len(cam) == 0:
        return 0.0
    h = len(cam) // 2
    return float((cam[h:].sum() - cam[:h].sum()) / len(cam))
    

def too_close(lidar, dir):
    R = 2 # Minimum circle radius for the car to turn
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
        self.step_count = 0

        self.internal_observation = {
            "speed_cmd": 0.0,
            "steering_cmd": 0.0,
        }
        # will be able to hold odometry derived applied_speed (most of the time different from wanted speed)
        # and applied_steering which can also be diff from wanted_steer if jitter / imprecisions

        self.crash_distance = np.sqrt(WIDTH_LIDAR_CAR**2 + LENGTH_LIDAR_CAR**2) + 0.05 # TODO : mesurer la marge d'erreur
        self.crash_angle = np.degrees(np.arctan2(WIDTH_LIDAR_CAR, LENGTH_LIDAR_CAR)) 
        
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
        fifo_dir = c.FIFO_DIR
        self.handler = logging.FileHandler(
            f"{fifo_dir}/vehicle_{self.simulation_rank}_{self.vehicle_rank}.log"
        )
        self.handler.setFormatter(c.FORMATTER)
        self.log = logging.getLogger(
            f"VEHICLE_{self.simulation_rank}_{self.vehicle_rank}"
        )
        self.log.setLevel(level=c.LOG_LEVEL)
        self.log.addHandler(self.handler)

        self.log.debug("Connection to the server")
        self.fifo_r = open(
            f"{fifo_dir}/serverto{self.simulation_rank}_{self.vehicle_rank}.pipe",
            "rb",
        )
        self.log.debug("Connection with the supervisor")
        self.fifo_w = open(
            f"{fifo_dir}/{self.simulation_rank}_{self.vehicle_rank}tosupervisor.pipe",
            "wb",
        )

        # Common Sense Check
        self.last_obs = None
        self.ref_sign = 0        
        self.calib_sum = 0.0
        self.calib_n = 0
        self.score_ema = 0.0
        self.wrong_count = 0
        self.ok_count = 0
        self.uturn_dir = 1
        self.uturn_reverse = False



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
            "Speed_Cmd": np.array([self.internal_observation["speed_cmd"]], dtype=np.float32),
            "Steer_Cmd": np.array([self.internal_observation["steering_cmd"]], dtype=np.float32),
        }

        # except:
        #     # In case of no lidar return
        #     return (
        #         np.array(self.touch_sensor.getValue(), dtype=np.float32),
        #         np.zeros(self.lidar.getNumberOfPoints(), dtype=np.float32),
        #     )

    # Step function of the GYM environment
    def step(self):
        self.step_count += 1
        # we cannot obs until an action has been applied,
        # otherwise, supervisor would have computed the reward for the previous action
        # then would have forwarded it to env that would compute the next action
        # so the reward would be for the previous action, not the current one
        # self.log.debug("Waiting for action from SERVER_{self.simulation_rank}_{self.vehicle_rank}")
        action_dtype = np.float32 if c.continuous_actions else np.int64
        try:
            action_data = read_exact(
                self.fifo_r,
                np.dtype(action_dtype).itemsize * 2,
                "Action from vehicle environment",
            )
        except FifoClosedError as exc:
            self.log.warning("%s; stopping vehicle controller", exc)
            return -1
        action = np.frombuffer(action_data, dtype=action_dtype)
        # self.log.info(f"received {action=}")

        self.check_direction()
        # Simulation step
        action_steering = action[0] * 0.4 if c.continuous_actions else np.linspace(
            -0.4, 0.4, c.n_actions_steering, dtype=np.float32
        )[action[0]]
        
        action_speed = (5.0 + float(action[1]) * 4.0) if c.continuous_actions else np.linspace(
            c.v_min, c.v_max, c.n_actions_speed, dtype=np.float32
            )[action[1]]
        
        cur_angle = self.getSteeringAngle()
        dt = self.getBasicTimeStep() / 1000.0
        omega = 20 # max angular speed of the steering wheel in rad/s

        action_steering = cur_angle + np.clip(action_steering - cur_angle, -omega * dt, omega * dt)

        self.setSteeringAngle(float(action_steering))
        self.setCruisingSpeed(float(action_speed))
        self.internal_observation["steering_cmd"] = float(action_steering)
        self.internal_observation["speed_cmd"] = float(action_speed)

        if self.state == State.BACK:
            self.back()

        if self.state == State.UTURN:
            self.uturn();
        
        if super().step() == -1:
            return -1

        obs = self.observe()
        self.last_obs = obs

        if self.crash() and self.state == State.AI: # TODO : Vérifier si le STATE.AI est utile ou non
            self.log.info("Crash détécté")
            self.state = State.BACK

        # self.log.info(f"observed {obs=}")
        self._send_observation(obs)
        return 0

    def _send_observation(self, obs: Dict[str, np.ndarray]):
        state = np.concatenate(
            [obs[key] for key in c.active_obs],
            axis=0,
        ).astype(np.float32, copy=False)
        self.fifo_w.write(state.tobytes())
        self.fifo_w.flush()

    def crash(self):
        if self.last_obs is None:
            return False

        # Touch sensor
        if self.last_obs["Collision"][0] > 0:
            return True

        # Lidar for the two front corners of the car
        lidar = np.clip(self.last_obs["Lidar"], 0, 10)
        n = len(lidar)

        def idx(angle_from_front):
            # Calculate the index in the lidar array of the point that is on the angle angle_from_front
            return int(round((FOV_DEG / 2 + angle_from_front) / FOV_DEG * (n - 1)))

        tol = int(n * ANGLE_TOLERANCE_CRASH / FOV_DEG) + 1
        i_g, i_d = idx(-self.crash_angle), idx(self.crash_angle)

        zones = np.concatenate((
            lidar[max(0, i_g - tol):i_g + tol + 1],
            lidar[max(0, i_d - tol):i_d + tol + 1],
        ))
        return zones.min() <= self.crash_distance

    def back(self):
        # if wall on "dir": turn to "dir" and reverse until able to move forward (wall distance to verify)
        lidar, cam = self.observe()["Lidar"], self.observe()["Cam"]

        S = sum(cam)
        dir = S > 0
        if dir:
            self.setSteeringAngle(0.33)
            if too_close(lidar, dir):
                self.setCruisingSpeed(V_BACK)
            else:
                self.state = State.AI
        else:
            self.setSteeringAngle(-0.33)
            if too_close(lidar, dir):
                self.setCruisingSpeed(V_BACK)
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
        self.last_obs = initial_obs
        # self.log.info(f"observed {initial_obs=}")
        self._send_observation(initial_obs)
        
        while self.step() != -1:
            pass

    def check_direction(self):
        if self.state != State.AI or self.last_obs is None: 
            return
        score = track_score(self.last_obs["Cam"])


        # Calibration at the beginning of the race
        if self.ref_sign == 0:
            self.calib_sum += score
            self.calib_n += 1
            if self.calib_n >= CALIB_STEPS:
                mean = self.calib_sum / self.calib_n
                self.ref_sign = 1 if mean > 0 else -1
                self.log.info(f"Sens de référence: {self.ref_sign}")

            return

        rel = score * self.ref_sign
        if rel < -SCORE_MIN:
            self.wrong_count += 1
        else:
            self.wrong_count = 0

        # Wrong direction
        if self.wrong_count >= WRONG_STEPS:
            lidar = np.clip(self.last_obs["Lidar"], 0, 10)
            h = len(lidar) // 2
            # Make a U-turn to the clearest side
            self.uturn_dir = 1 if lidar[h:].mean() > lidar[:h].mean() else -1
            self.uturn_reverse = False
            self.ok_count = 0
            self.state = State.UTURN
            self.log.info("Mauvaise direction détéctée")

    def uturn(self):
        if self.last_obs is None:
            return
        lidar = np.clip(self.last_obs["Lidar"], 0, 10)
        rel = track_score(self.last_obs["Cam"]) * self.ref_sign

        self.ok_count = self.ok_count + 1 if rel > SCORE_MIN else 0
        if self.ok_count >= UTURN_OK_STEPS:
            self.score_ema, self.wrong_count = 0.0, 0
            self.state = State.AI
            return

        front = lidar[len(lidar) // 2]
        if self.uturn_reverse:
            self.setSteeringAngle(-0.33 * self.uturn_dir)
            self.setCruisingSpeed(-1 * V_UTURN)
            if front > 1.0:
                self.uturn_reverse = False
        else:
            self.setSteeringAngle(0.33 * self.uturn_dir)
            self.setCruisingSpeed(V_UTURN)
            if front < 0.5:
                self.uturn_reverse = True

# ----------------Main Program--------------------
def main():
    driver = VehicleDriver()
    driver.log.info("Starting the vehicle driver")
    driver.run()


if __name__ == "__main__":
    main()
