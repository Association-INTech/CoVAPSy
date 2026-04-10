import logging
import os
import time
from threading import Thread
from typing import Optional

import numpy as np

from drivers import Lidar
from drivers.camera import Camera
from drivers.tof import ToF

# Import constants from HL.Autotech_constant to share them between files and ease of use
from high_level.autotech_constant import (
    BACKWARD_IA_SPEED,
    LIDAR_DATA_AMPLITUDE,
    LIDAR_DATA_OFFSET,
    LIDAR_DATA_SIGMA,
    MAX_IA_SPEED,
    MAX_ANGLE,
    MIN_ANGLE,
    MODEL_PATH,
    LIMIT_CRASH_POINT,
    FREQUENCY_CRASH_DETECTION,
)
from programs.camera_proxy import CameraProxy

from .program import Program
from .utils import Driver


def too_close(lidar_m, dir):
    R = 0.83
    lidar_m = np.asarray(lidar_m, dtype=np.float32)

    if lidar_m.size == 0:
        return True

    length = len(lidar_m)
    straight = np.average(
        lidar_m[length // 2 - 10 : length // 2 + 10]
    )  # take the average of 20 points around the middle to reduce noise

    zone = lidar_m[length // 2 :] if dir else lidar_m[: length // 2]

    # We keep only valid distances (greater than 0 and finite) for the nearest calculation to avoid issues with invalid lidar readings. If there are no valid readings, we consider it as too close to be safe.
    valid_zone = zone[np.isfinite(zone) & (zone > 0)]

    if valid_zone.size == 0:
        return True

    nearest = np.average(
        np.sort(valid_zone)[:10]
    )  # take the average of the 10 nearest points to reduce noise

    # straight can also be invalid, we consider it as too close in that case to be safe
    if not np.isfinite(straight) or straight <= 0:
        return True

    cos = nearest / straight

    # wwe never know
    if not np.isfinite(cos):
        return True

    cos = np.clip(cos, -1.0, 1.0)

    theta = np.arccos(cos)
    L = R * (1 - np.sin(theta))
    return nearest < L


class Border_zone:
    ZONE1 = [750, 1080]
    ZONE2 = [540, 749]
    ZONE3 = [330, 540]
    ZONE4 = [0, 330]


class CrashCar:
    def __init__(self, server) -> None:
        self.log = logging.getLogger(__name__)
        self.server = server
        self.crashed = False
        self.state = 0
        self.deadzone = 40  # mm Distance under which the car considers that it is in a crash situation even if the lidar contour is not penetrated, to avoid issues with the lidar contour not being perfectly accurate and to add a safety margin
        # Load reference lidar contour once
        try:
            self.reference_lidar = np.load(
                "/home/intech/CoVAPSy/src/high_level/src/programs/data/min_lidar.npy"
            )[
                :1080
            ]  # Load only the first 1080 values to match the lidar data used by the AI
            self.log.info("Reference lidar contour loaded")
            Thread(target=self.has_Crashed, daemon=True).start()
        except Exception as e:
            self.log.error(f"Unable to load reference lidar contour: {e}")
            raise

    def has_Crashed(self) -> bool:

        while self.server.lidar is None:
            self.log.debug("Lidar not yet ready for crash detection")
            time.sleep(0.1)
        while True:
            current = self.server.lidar.r_distance[:1080]

            if current is None or len(current) != len(self.reference_lidar):
                self.log.warning(
                    str(len(current) if current is not None else "None")
                    + " lidar points received, expected "
                    + str(len(self.reference_lidar))
                )
                self.crashed = False
            else:
                # Points that are inside the vehicle contour
                penetration_mask = (current > 0) & (
                    current < self.reference_lidar + self.deadzone
                )

                penetration_count = np.sum(penetration_mask)

                self.log.debug(f"Penetration points: {penetration_count}")

                if penetration_count >= LIMIT_CRASH_POINT:
                    # self.log.info("Crash detected via contour penetration")
                    self.crashed = True
                else:
                    self.crashed = False
            time.sleep(FREQUENCY_CRASH_DETECTION)  # time between two crash detection


class Car:
    def __init__(self, driving_strategy, server) -> None:
        self.log = logging.getLogger(__name__)
        self.target_speed = 0  # Speed in millimeters per second
        self.direction = 0  # Steering angle in degrees
        self.server = server
        self.reverse_count = 0
        self.driving = driving_strategy
        self.state = 0
        self.crash_time = time.time()

        self.log.info("Car initialization complete")

    # dynamic access to sensors
    @property
    def camera(self) -> CameraProxy:
        return self.server.camera

    @property
    def lidar(self) -> Lidar:
        return self.server.lidar

    @property
    def tof(self) -> ToF:
        return self.server.tof

    def stop(self) -> None:
        self.target_speed = 0
        self.direction = 0
        self.log.info("Motor stop")

    def turn_around(self) -> None:
        """Turn the car around."""
        self.log.info("Turning around")

        self.target_speed = 0
        self.direction = MAX_ANGLE
        self.target_speed = -2  # blocing call
        time.sleep(1.8)  # Wait for the car to turn around
        if self.server.camera_red_or_green.is_reverse:
            self.turn_around()

    def back(self, lidar_m, cam):
        # if wall on "dir": turn to "dir" and reverse until able to move forward (wall distance to verify)
        # turn to the right
        if self.tof.distance < 20:
            self.log.info("Obstacle detected by ToF")
            self.state = 0
            return 0, 0

        zone1 = np.average(lidar_m[Border_zone.ZONE1[0] : Border_zone.ZONE1[1]])
        zone2 = np.average(lidar_m[Border_zone.ZONE2[0] : Border_zone.ZONE2[1]])
        zone3 = np.average(lidar_m[Border_zone.ZONE3[0] : Border_zone.ZONE3[1]])
        zone4 = np.average(lidar_m[Border_zone.ZONE4[0] : Border_zone.ZONE4[1]])

        zones = [zone1, zone2, zone3, zone4]

        id_min_zone = np.argmin(zones)
        # print("id_min_zone =", id_min_zone)

        if id_min_zone == 0:
            return MAX_ANGLE, MAX_IA_SPEED
        if id_min_zone == 3:
            return MIN_ANGLE, MAX_IA_SPEED

        if id_min_zone == 1:
            dir = sum(cam[: len(cam) // 2]) > 0
            if dir:
                return MIN_ANGLE, BACKWARD_IA_SPEED

            if too_close(lidar_m, dir):
                return MIN_ANGLE, BACKWARD_IA_SPEED
            else:
                self.state = 0
                return MAX_ANGLE, BACKWARD_IA_SPEED

        if id_min_zone == 2:
            dir = sum(cam[: len(cam) // 2]) > 0
            if not dir:
                return MAX_ANGLE, BACKWARD_IA_SPEED

            if too_close(lidar_m, dir):
                return MAX_ANGLE, BACKWARD_IA_SPEED
            else:
                self.state = 0
                return MIN_ANGLE, BACKWARD_IA_SPEED
        return 0, 0

        # S = sum(cam)

        # dir = S > 0
        # self.log.info("je suis en arriere")
        # if dir:
        #     # turn to the right
        #     if too_close(lidar_m, dir):
        #         return MAX_ANGLE, BACKWARD_IA_SPEED
        #     else:
        #         self.state = 0
        #         return (
        #             MAX_ANGLE,
        #             BACKWARD_IA_SPEED,
        #         )  # the ai takes distances in meters not in mm
        # else:
        #     if too_close(lidar_m, dir):
        #         return MIN_ANGLE, BACKWARD_IA_SPEED
        #     else:
        #         self.state = 0
        #         return MIN_ANGLE, BACKWARD_IA_SPEED

    def reverse_car(self):
        zone_gauche = self.lidar.r_distance[Border_zone.ZONE1[0] : Border_zone.ZONE1[1]]
        zone_gauche_valid = zone_gauche[np.isfinite(zone_gauche) & (zone_gauche > 0)]
        zone_droite = self.lidar.r_distance[Border_zone.ZONE3[0] : Border_zone.ZONE3[1]]
        zone_droite_valid = zone_droite[np.isfinite(zone_droite) & (zone_droite > 0)]
        nearest_droite = np.average(
            np.sort(zone_droite_valid)[:10]
        )  # take the average of the 10 nearest points to reduce noise
        nearest_gauche = np.average(
            np.sort(zone_gauche_valid)[:10]
        )  # take the average of the 10 nearest points to reduce noise

        if nearest_droite < nearest_gauche:
            direction_1 = MIN_ANGLE
            direction_2 = MAX_ANGLE
        else:
            direction_1 = MAX_ANGLE
            direction_2 = MIN_ANGLE
        self.log.info("Reversing due to red light")

        self.target_speed = BACKWARD_IA_SPEED
        self.direction = direction_1
        t = time.time()
        ok = True
        self.log.info(
            "reversing for 1.5 seconds or until an obstacle is detected behind the car"
        )
        while (
            time.time() - t < 2 and self.tof.distance > 20 and ok
        ):  # reverse for a maximum of 1.5 seconds or until an obstacle is detected behind the car
            if time.time() - t > 1 and self.server.arduino_I2C.current_speed == 0:
                ok = False

        self.log.info("Stopped reversing after 1.5 seconds")

        self.target_speed = MAX_IA_SPEED
        self.direction = direction_2
        t = time.time()
        straight_mm = np.average(
            self.lidar.r_distance[
                len(self.lidar.r_distance) // 2 - 10 : len(self.lidar.r_distance) // 2
                + 10
            ]
        )
        while (
            time.time() - t < 1 and straight_mm > 50
        ):  # reverse for a maximum of 1.5 seconds or until an obstacle is detected behind the car
            straight_mm = np.average(
                self.lidar.r_distance[
                    len(self.lidar.r_distance) // 2 - 10 : len(self.lidar.r_distance)
                    // 2
                    + 10
                ]
            )  # take the average of 20 points around the middle to reduce noise
            pass
        self.log.info("Stopped moving forward after 1 second")

    def main(self) -> None:
        # retrieve lidar data. We only take the first 1080 values and ignore the last one for simplicity for the ai
        if self.camera is None or self.lidar is None:
            self.log.debug("Sensors not yet ready")
            return
        lidar_data = self.lidar.r_distance.copy()  # convert to meters
        # print(
        #     "len(np.where(lidar_data == 0)[0]) =", len(np.where(lidar_data_m == 0)[0])
        # )
        lidar_data_m = lidar_data / 1000
        # convert to meters and add Gaussian noise. We manipulate the data provided to the AI
        camera_data = self.camera.camera_matrix()  # retrieve camera data

        if self.server.camera_red_or_green.is_reverse:
            self.reverse_car()

        if (
            self.server.crash_car.crashed
            and self.server.arduino_I2C.current_speed < 1000
        ):
            self.crash_time = time.time()
            self.state = 1

        if self.state == 1:
            self.direction, self.target_speed = self.back(lidar_data_m, camera_data)
            if time.time() - self.crash_time > 2:
                self.crash_time = time.time()
                self.state = 0
        else:
            self.direction, self.target_speed = self.driving(lidar_data_m, camera_data)

        self.log.debug(
            f"Min Lidar: {min(lidar_data_m)}, Max Lidar: {max(lidar_data_m)}"
        )

        """
        if self.camera.is_running_in_reversed():
            self.reverse_count += 1
        else:
            self.reverse_count = 0
        if self.reverse_count > 2:
            self.turn_around()
            self.reverse_count = 0

        if self.server.crash_car.crashed:
            self.log.info("Obstacle detected")
            color = self.camera.is_green_or_red(lidar_data)
            if color == 0:
                small_distances = [
                    d for d in self.lidar.r_distance if 0 < d < CRASH_DIST
                ]
                if len(small_distances) == 0:
                    self.log.info("No obstacle detected")
                    return
                min_index = np.argmin(small_distances)
                direction = (
                    MAX_ANGLE if min_index < 540 else -MAX_ANGLE
                )  # 540 is the middle of the lidar
                color = direction / direction
                self.log.info("Obstacle detected, Lidar Fallback")
            if color == -1:
                self.log.info("Red obstacle detected")
            if color == 1:
                self.log.info("Green obstacle detected")
            angle = -color * MAX_ANGLE
            self.target_speed = -2
            self.direction = angle"""


class AIProgram(Program):
    def __init__(self, server) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.server = server
        self.driver = None
        self.GR86 = None
        self.running = False
        self.controls_car = True
        try:
            self.models = [
                model for model in os.listdir(MODEL_PATH) if model.endswith(".onnx")
            ]
        except Exception as e:
            self.log.error(f"Error retrieving models: {e}")
        self.nb_models = len(self.models)
        self.id_model = self.nb_models
        # start with the last model which is the fallback for not running

    @property
    def target_speed(self) -> float:
        if self.GR86 is None:
            return 0.0
        return self.GR86.target_speed

    @property
    def direction(self) -> float:
        if self.GR86 is None:
            return 0.0
        return self.GR86.direction

    def run(self) -> None:
        while self.running:
            try:
                if self.GR86 is not None:
                    self.GR86.main()
            except Exception as e:
                self.log.error(f"AI error: {e}")
                self.running = False
                raise

    def initializeai(self, model: str) -> None:
        self.driver = Driver()
        self.driver.load_model(model)

        driving_strategy = self.driver.omniscent

        self.GR86 = Car(driving_strategy, self.server)

    def start(self, model_give: Optional[str] = None) -> None:

        if self.server.camera is None or self.server.lidar is None:
            self.log.error("Sensors not initialized")
            return
        if self.models is None:
            self.log.error("No models available for AI")
            return
        if model_give is not None:
            self.initializeai(model_give)
            self.log.info(f"Starting AI with model {model_give}")
        else:
            try:
                self.id_model = (self.id_model + 1) % (self.nb_models)
                self.log.info(f"Selected model: {self.models[self.id_model]}")
                model = self.models[self.id_model]
                self.initializeai(model)

            except Exception as e:
                self.log.error(f"Unable to start AI: {e}")
                self.driver = None
                self.GR86 = None
                return

        self.running = True
        Thread(target=self.run, daemon=True).start()

    def kill(self) -> None:
        self.running = False

    def display(self) -> str:
        text = self.__class__.__name__
        if self.running:
            text += "*"
        if not self.running:
            for model in self.models:
                text += f"\n   {model[:8]}..."  # display only the first 5 characters of the model name to avoid cuttoff

        else:
            for model in self.models:
                if model == self.models[self.id_model]:
                    text += f"\n -> {model[:8]}..."
                else:
                    text += f"\n   {model[:8]}..."
        return text


"""
if __name__ == '__main__': # non functional
    Format= '%(asctime)s:%(name)s:%(levelname)s:%(message)s'
    if input("Press D to start in debug mode or any other key to start in normal mode") in ("D", "d"):
        logging.basicConfig(level=logging.DEBUG, format=Format)
    else:
        logging.basicConfig(level=logging.INFO, format=Format)
    bp2 = Button("GPIO6")
    try:
        Schumacher = Driver(128, 128)
        GR86 = Car(Schumacher,None,None)
        GR86._initialize_camera()
        GR86._initialize_lidar()
        logging.info("Initialization complete")
        if input("Press D to start or any other key to quit") in ("D", "d") or bp2.is_pressed:
            logging.info("Start")
            while True:
                GR86.main()
        else:
            raise Exception("The program was stopped by the user")
    except KeyboardInterrupt:
        GR86.stop()
        logging.info("The program was stopped by the user")

    except Exception as e: # catch all exceptions to stop the car
        GR86.stop()
        logging.error("Unknown error")
        raise e # re-raise the exception to see the error message
    """
