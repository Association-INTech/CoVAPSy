import logging
import os
import time
from threading import Thread
from typing import Optional

import numpy as np
import onnxruntime as ort

from driver import Lidar
from driver.camera import Camera
from driver.tof import ToF

# Import constants from HL.Autotech_constant to share them between files and ease of use
from high_level.autotech_constant import (
    LIDAR_DATA_AMPLITUDE,
    LIDAR_DATA_OFFSET,
    LIDAR_DATA_SIGMA,
    MAX_ANGLE,
    MODEL_PATH,
    LIMIT_CRASH_POINT,
    FREQUENCY_CRASH_DETECTION,
)

from .program import Program
from .utils import Driver


class Border_zone:
    ZONE1 = [0, 370]
    ZONE2 = [371, 750]
    ZONE3 = [751, 1080]


class CrashCar:
    def __init__(self, serveur) -> None:
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
        self.crashed = False
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

        while self.serveur.lidar is None:
            self.log.debug("Lidar not yet ready for crash detection")
            time.sleep(0.1)
        while True:
            current = self.serveur.lidar.r_distance[:1080]

            if current is None or len(current) != len(self.reference_lidar):
                self.log.warning(
                    str(len(current) if current is not None else "None")
                    + " lidar points received, expected "
                    + str(len(self.reference_lidar))
                )
                self.crashed = False
            else:
                # Points that are inside the vehicle contour
                penetration_mask = (current > 0) & (current < self.reference_lidar)

                penetration_count = np.sum(penetration_mask)

                self.log.debug(f"Penetration points: {penetration_count}")

                if penetration_count >= LIMIT_CRASH_POINT:
                    self.log.info("Crash detected via contour penetration")
                    self.crashed = True
                else:
                    self.crashed = False
            time.sleep(FREQUENCY_CRASH_DETECTION)  # time between two crash detection


class Car:
    def __init__(self, driving_strategy, serveur, model) -> None:
        self.log = logging.getLogger(__name__)
        self.target_speed = 0  # Speed in millimeters per second
        self.direction = 0  # Steering angle in degrees
        self.serveur = serveur
        self.reverse_count = 0

        # Initialize AI session
        try:
            self.ai_session = ort.InferenceSession(MODEL_PATH)
            self.log.info("AI session initialized successfully")
        except Exception as e:
            self.log.error(f"Error initializing AI session: {e}")
            raise

        self.driving = driving_strategy

        self.log.info("Car initialization complete")

    # dynamic access to sensors
    @property
    def camera(self) -> Camera:
        return self.serveur.camera

    @property
    def lidar(self) -> Lidar:
        return self.serveur.lidar

    @property
    def tof(self) -> ToF:
        return self.serveur.tof

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
        if self.camera.is_running_in_reversed():
            self.turn_around()

    def main(self) -> None:
        # retrieve lidar data. We only take the first 1080 values and ignore the last one for simplicity for the ai
        if self.camera is None or self.lidar is None:
            self.log.debug("Sensors not yet ready")
            return
        lidar_data = self.lidar.r_distance[:1080] / 1000
        lidar_data_ai = (
            (lidar_data - 0.5)
            * (
                LIDAR_DATA_OFFSET
                + LIDAR_DATA_AMPLITUDE
                * np.exp(-1 / 2 * ((np.arange(1080) - 135) / LIDAR_DATA_SIGMA**2))
            )
        )  # convert to meters and add Gaussian noise. We manipulate the data provided to the AI
        self.direction, self.target_speed = self.driving(
            lidar_data_ai, self.camera.camera_matrix()
        )  # the ai takes distances in meters not in mm
        self.log.debug(f"Min Lidar: {min(lidar_data)}, Max Lidar: {max(lidar_data)}")
        """
        if self.camera.is_running_in_reversed():
            self.reverse_count += 1
        else:
            self.reverse_count = 0
        if self.reverse_count > 2:
            self.turn_around()
            self.reverse_count = 0

        if self.serveur.crash_car.crashed:
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
    def __init__(self, serveur) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
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
                print("lolooibiiuib : " + self.running.__str__())
            except Exception as e:
                self.log.error(f"AI error: {e}")
                self.running = False
                raise

    def initializeai(self, model: str) -> None:
        self.driver = Driver(128, 128)
        self.driver.load_model(model)

        # self.GR86 = Car(self.driver.ai, self.serveur, model)
        self.GR86 = Car(self.driver.omniscent, self.serveur, model)
        # self.GR86 = Car(self.driver.simple_minded, self.serveur, model)

    def start(self, model_give: Optional[str] = None) -> None:

        if self.serveur.camera is None or self.serveur.lidar is None:
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
