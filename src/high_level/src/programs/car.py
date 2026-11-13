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
    LIMIT_CRASH_RATIO,
    FREQUENCY_CRASH_DETECTION,
    BOX_REAR_X,
    X_WHEELS,
    BOX_FRONT_X,
    BOX_HALF_WIDTH,
    ZONE_FRONT_HALF_WIDTH,
    CRASH_MARGIN,
    FOV_DEG,
    LIDAR_X,
    CRASH_SIDE_SPEED,
    CRASH_SIDE_STEER_TIME,
    CRASH_SIDE_STRAIGHT_TIME,
    CRASH_CORNER_BACK_TIME,
    CRASH_CORNER_REALIGN_TIME,
    CRASH_FRONT_BACK_TIME,
    CRASH_FRONT_REALIGN_TIME,
    CRASH_COOLDOWN,
    CRASH_MAX_DURATION,
    TOF_REAR_STOP,
)
GREEN_WALL_SIDE = 1 # +1 = gauche, -1 = droite, TODO : le calculer manuellement grace a la camera au debut de la course

from programs.camera_proxy import CameraProxy, COLOUR_KEY

from .program import Program
from .utils import Driver


class CrashCar:
    ZONE_0 = {  # côté droit
        "x_min": BOX_REAR_X - CRASH_MARGIN, "x_max": X_WHEELS,
        "y_min": -(BOX_HALF_WIDTH + CRASH_MARGIN), "y_max": -(BOX_HALF_WIDTH - CRASH_MARGIN),
    }
    ZONE_1 = {  # coin avant droit (roue)
        "x_min": X_WHEELS, "x_max": BOX_FRONT_X + CRASH_MARGIN,
        "y_min": -(BOX_HALF_WIDTH + CRASH_MARGIN), "y_max": -ZONE_FRONT_HALF_WIDTH,
    }
    ZONE_2 = {  # avant
        "x_min": X_WHEELS, "x_max": BOX_FRONT_X + CRASH_MARGIN,
        "y_min": -ZONE_FRONT_HALF_WIDTH, "y_max": ZONE_FRONT_HALF_WIDTH,
    }
    ZONE_3 = {  # coin avant gauche (roue)
        "x_min": X_WHEELS, "x_max": BOX_FRONT_X + CRASH_MARGIN,
        "y_min": ZONE_FRONT_HALF_WIDTH, "y_max": BOX_HALF_WIDTH + CRASH_MARGIN,
    }
    ZONE_4 = {  # côté gauche
        "x_min": BOX_REAR_X - CRASH_MARGIN, "x_max": X_WHEELS,
        "y_min": BOX_HALF_WIDTH - CRASH_MARGIN, "y_max": BOX_HALF_WIDTH + CRASH_MARGIN,
    }

    ZONES = (ZONE_0, ZONE_1, ZONE_2, ZONE_3, ZONE_4)
    ZONE_SEVERITY = (2, 1, 0, 1, 2)

    def __init__(self, server) -> None:
        self.log = logging.getLogger(__name__)
        self.server = server
        self.crashed = False
        self.crash_zone = None
        self.state = 0

        n = 1080
        ang = np.radians(-FOV_DEG / 2 + np.arange(n) * FOV_DEG / (n - 1))
        self._cos, self._sin = np.cos(ang), np.sin(ang)

        # nombre de rayons qui traversent chaque zone
        t = np.arange(0.0, 1.0, 0.005, dtype=np.float32) # distances le long du rayon (m)
        px = LIDAR_X + self._cos[:, None] * t[None, :] # (1080, nb_pas)
        py = self._sin[:, None] * t[None, :]
        self._rays_per_zone = np.array([
            np.count_nonzero(
                ((px >= z["x_min"]) & (px <= z["x_max"])
                & (py >= z["y_min"]) & (py <= z["y_max"])).any(axis=1)
            )
            for z in self.ZONES
        ], dtype=np.float32)

        self.log.info(f"Rays per zone: {self._rays_per_zone.tolist()}")
        self._xmin = np.array([z["x_min"] for z in self.ZONES])[:, None]
        self._xmax = np.array([z["x_max"] for z in self.ZONES])[:, None]
        self._ymin = np.array([z["y_min"] for z in self.ZONES])[:, None]
        self._ymax = np.array([z["y_max"] for z in self.ZONES])[:, None]

        Thread(target=self.has_Crashed, daemon=True).start()

    def detect(self, r):
        r = np.asarray(r, dtype=np.float32) / 1000.0 # lidar mm -> m
        if r.size != self._cos.size:
            return False, None
        ok = np.isfinite(r) & (r > 0)
        x = r * self._cos + LIDAR_X
        y = r * self._sin
        inside = (
            (x >= self._xmin) & (x <= self._xmax)
            & (y >= self._ymin) & (y <= self._ymax)
            & ok
        )
        counts = inside.sum(axis=1)
        ratios = counts / np.maximum(self._rays_per_zone, 1)

        valid = ratios >= LIMIT_CRASH_RATIO
        if not valid.any():
            return False, None

        best = min(
            (i for i in range(len(self.ZONES)) if valid[i]),
            key=lambda i: (-ratios[i], self.ZONE_SEVERITY[i]),
        )
        self.log.debug(f"Crash zone {best}: ratios={np.round(ratios, 2).tolist()}")
        return True, best


    def has_Crashed(self) -> None:
        while self.server.lidar is None:
            self.log.debug("Lidar not yet ready for crash detection")
            time.sleep(0.1)
        while True:
            try:
                crashed, zone = self.detect(self.server.lidar.r_distance)
            except Exception as e:
                self.log.error(f"Crash detection error: {e}")
                crashed, zone = False, None
            self.crash_zone = zone
            self.crashed = crashed
            time.sleep(FREQUENCY_CRASH_DETECTION)  # time between two crash detections


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
        self.times_not_working = time.time()
        self.stuck = 0
        self.not_working = False
        self.maneuver = [] # liste d'étapes (direction, vitesse, durée)
        self.maneuver_step = 0
        self.maneuver_start = 0.0
        self.step_start = 0.0
        self.last_maneuver_end = 0.0
        self.wall_color_at_crash = COLOUR_KEY["none"]

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

    def plan_maneuver(self, zone, lidar_m):
        """Retourne une liste d'étapes (direction, vitesse, durée)."""
        fwd, back = CRASH_SIDE_SPEED, BACKWARD_IA_SPEED

        # Zones 0 / 4 : mur sur le flanc -> braquage rapide en avançant, puis roues droites
        if zone == 0:  # mur à gauche -> on braque à droit
            return [(RIGHT, fwd, CRASH_SIDE_STEER_TIME),
                    (0, fwd, CRASH_SIDE_STRAIGHT_TIME)]
        if zone == 4:  # mur à droit -> on braque à gauche
            return [(LEFT, fwd, CRASH_SIDE_STEER_TIME),
                    (0, fwd, CRASH_SIDE_STRAIGHT_TIME)]

        # Zones 1 / 3 : coin avant -> on recule braqué (l'avant s'écarte du mur),
        # puis on recule roues dans l'autre sens pour se réaligner
        if zone == 1:  # coin avant gaucge : en marche arrière roues à gauche, l'avant part à droit
            return [(LEFT, back, CRASH_CORNER_BACK_TIME),
                    (RIGHT, back, CRASH_CORNER_REALIGN_TIME)]
        if zone == 3:
            return [(RIGHT, back, CRASH_CORNER_BACK_TIME),
                    (LEFT, back, CRASH_CORNER_REALIGN_TIME)]

        # Zone 2 : mur de face -> on utilise la couleur
        if zone == 2:
            color = self.wall_color_at_crash
            if color != COLOUR_KEY["none"]:
                # côté où le mur doit rester par rapport à nous (+1 gauche, -1 droite)
                wall_side = (
                    GREEN_WALL_SIDE if color == COLOUR_KEY["green"] else -GREEN_WALL_SIDE
                )
                # en marche arrière, roues du côté du mur => l'avant part de l'autre côté
                first = LEFT if wall_side > 0 else RIGHT
            else:
                # fallback lidar : l'avant part vers le côté le plus libre
                lidar_m = np.asarray(lidar_m, dtype=np.float32)
                half = len(lidar_m) // 2
                right = lidar_m[:half]
                left = lidar_m[half:]
                right = right[np.isfinite(right) & (right > 0)]
                left = left[np.isfinite(left) & (left > 0)]
                r_free = float(np.mean(right)) if right.size else 0.0
                l_free = float(np.mean(left)) if left.size else 0.0
                first = RIGHT if l_free > r_free else LEFT
            second = LEFT if first == RIGHT else RIGHT
            return [(first, back, CRASH_FRONT_BACK_TIME),
                    (second, back, CRASH_FRONT_REALIGN_TIME)]

        return []


    def start_maneuver(self, zone, lidar_m):
        now = time.time()
        self.wall_color_at_crash = COLOUR_KEY["none"]
        if zone == 2:
            try:
                self.wall_color_at_crash = self.camera.wall_color()
            except Exception as e:
                self.log.error(f"Wall color unavailable: {e}")
        self.maneuver = self.plan_maneuver(zone, lidar_m)
        self.log.info(
            f"Crash zone {zone}, color={self.wall_color_at_crash}, plan={self.maneuver}"
        )
        if not self.maneuver:
            return
        self.maneuver_step = 0
        self.maneuver_start = self.step_start = now
        self.state = 1

    def end_maneuver(self):
        self.state = 0
        self.maneuver = []
        self.last_maneuver_end = time.time()

    def run_maneuver(self):
        now = time.time()
        if now - self.maneuver_start > CRASH_MAX_DURATION:
            self.log.warning("Maneuver timeout")
            self.end_maneuver()
            return 0, 0

        direction, speed, duration = self.maneuver[self.maneuver_step]
        # sécurité arrière : uniquement quand on recule
        if speed < 0 and self.tof.distance < TOF_REAR_STOP:
            self.log.info("Obstacle detected by ToF")
            self.end_maneuver()
            return 0, 0

        if now - self.step_start > duration:
            self.maneuver_step += 1
            if self.maneuver_step >= len(self.maneuver):
                self.end_maneuver()
                return 0, 0
            self.step_start = now
            direction, speed, _ = self.maneuver[self.maneuver_step]
        return direction, speed

"""
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

        if nearest_droite > nearest_gauche:
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
            time.time() - t < 2
            and self.tof.distance > 20
            and ok
            and not self.server.camera_red_or_green.is_not_reverse
        ):  # reverse for a maximum of 1.5 seconds or until an obstacle is detected behind the car
            if time.time() - t > 1 and self.server.arduino_I2C.current_speed == 0:
                ok = False

        self.log.info("Stopped reversing after 1.5 seconds")

        self.target_speed = 1000
        self.direction = direction_2
        t = time.time()
        straight_mm = np.average(
            self.lidar.r_distance[
                len(self.lidar.r_distance) // 2 - 10 : len(self.lidar.r_distance) // 2
                + 10
            ]
        )
        while (
            time.time() - t < 1
            and straight_mm > 50
            and not self.server.camera_red_or_green.is_not_reverse
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
        lidar_data = self.lidar.r_distance.copy()
        lidar_data_m = lidar_data / 1000  # convert to meters
        camera_data = self.camera.camera_matrix()  # retrieve camera data

        if self.server.arduino_I2C.current_speed == 0 and not self.not_working:
            self.times_not_working = time.time()
            self.not_working = True

        if self.server.arduino_I2C.current_speed > 0:
            self.not_working = False


        if self.server.camera_red_or_green.is_reverse:
            self.reverse_car()

        crash_car = self.server.crash_car
        crash_zone = crash_car.crash_zone  # lu une seule fois (le thread peut la changer)

        if (
            self.state == 0
            and crash_car.crashed
            and crash_zone is not None
            and self.server.arduino_I2C.current_speed < 1000
            and time.time() - self.last_maneuver_end > CRASH_COOLDOWN
        ):
            self.start_maneuver(crash_zone, lidar_data_m)

        if self.state == 1:
            self.direction, self.target_speed = self.run_maneuver()
        else:
            self.direction, self.target_speed = self.driving(lidar_data_m, camera_data)

        self.log.debug(
            f"Min Lidar: {min(lidar_data_m)}, Max Lidar: {max(lidar_data_m)}"
        )
"""

class AIProgram(Program):
    def __init__(self, server) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.server = server
        self.driver = None
        self.GR86 = None
        self.running = False
        self.controls_car = True
        self.control_site = False
        try:
            self.models = [
                model for model in os.listdir(MODEL_PATH) if model.endswith(".onnx")
            ]
        except Exception as e:
            self.log.error(f"Error retrieving models: {e}")
            self.models = []
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
        self.model = model
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
            self.control_site = True
            return
        elif not self.control_site:  # if the site is not controlling the car, we select the model ourselves else we wait for the site to select the model to avoid conflicts
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
                text += f"\n   {model[:8]}..."  # display only the first 8 characters of the model name to avoid cuttoff

        else:
            for model in self.models:
                if model == self.models[self.id_model % self.nb_models]:
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
