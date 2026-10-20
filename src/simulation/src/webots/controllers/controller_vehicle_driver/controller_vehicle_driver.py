import logging
from enum import Enum, auto
from typing import Tuple, cast

import numpy as np
from controller import Camera, Lidar, TouchSensor
from vehicle import Driver

from simulation import config as c
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

        self.v_min = V_MIN
        self.v_max = V_MAX
        self.step_count = 0

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
    def observe(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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

        return (sensor_data, lidar_data, camera_data)

        # except:
        #     # In case of no lidar return
        #     return (
        #         np.array(self.touch_sensor.getValue(), dtype=np.float32),
        #         np.zeros(self.lidar.getNumberOfPoints(), dtype=np.float32),
        #     )

    # Step function of the GYM environment
    def step(self):
        self.step_count += 1
        match self.state:
            case State.AI:
                self.ai()
            case State.BACK:
                # TODO: this is a very poor fix and not a definitive solution
                # we HAVE to remove this line and properly manage the communication
                # in State.BACK so that the ai doesn't train the data during that
                # period to make sure that the training is not polluted
                self.ai()
                self.back()
            case State.UTURN:
                # TODO: this is a very poor fix and not a definitive solution
                # we HAVE to remove this line and properly manage the communication
                # in State.UTURN so that the ai doesn't train the data during that
                # period to make sure that the training is not polluted
                self.ai()
                self.uturn()

        return super().step()

    def ai(self):
        self.log.info("Starting step")
        sensor_data, lidar_data, camera_data = self.observe()
        obs = (sensor_data, lidar_data, camera_data)
        self.last_obs = obs
        self.log.info(f"Observe {obs=}")
        self.fifo_w.write(np.concatenate(obs).tobytes())
        self.fifo_w.flush()

        self.log.info(f"Waiting for an action from SERVER_{self.simulation_rank}_{self.vehicle_rank}")
        action = np.frombuffer(
            self.fifo_r.read(np.dtype(np.int64).itemsize * 2), dtype=np.int64
        )
        self.log.info(f"received {action=}")

        self.check_direction()

        # Simulation step
        action_steering = np.linspace(
            -0.4, 0.4, c.n_actions_steering, dtype=np.float32
        )[action[0]]

        action_speed = np.linspace(
            self.v_min, self.v_max, c.n_actions_speed, dtype=np.float32
        )[action[1]]

        cur_angle = self.getSteeringAngle()
        dt = self.getBasicTimeStep()
        omega = 20  # rad/s (max angular speed of the steering servo)

        action_steering = cur_angle + np.clip(
            action_steering - cur_angle, -omega * dt, omega * dt
        )

        self.setSteeringAngle(float(action_steering))
        self.setCruisingSpeed(float(action_speed))

        if self.crash() and self.state == State.AI: # TODO : Vérifier si le STATE.AI est utile ou non
            self.log.info("Crash détécté")
            self.state = State.BACK

    def crash(self):
        # Touch sensor
        if self.last_obs[0][0] > 0:
            return True

        # Lidar for the two front corners of the car
        lidar = np.clip(self.last_obs[1], 0, 10)
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
        lidar, cam = self.observe()[1:]
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
        super().step()
        while self.step() != -1:
            pass

    def check_direction(self):
        if self.state != State.AI: 
            return
        score = track_score(self.last_obs[2])


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
            lidar = np.clip(self.last_obs[1], 0, 10)
            h = len(lidar) // 2
            # Make a U-turn to the clearest side
            self.uturn_dir = 1 if lidar[h:].mean() > lidar[:h].mean() else -1
            self.uturn_reverse = False
            self.ok_count = 0
            self.state = State.UTURN
            self.log.info("Mauvaise direction détéctée")

    def uturn(self):
        lidar = np.clip(self.last_obs[1], 0, 10)
        rel = track_score(self.last_obs[2]) * self.ref_sign

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
