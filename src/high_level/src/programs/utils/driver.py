import logging
import math
from typing import Any, Tuple, cast

import numpy as np
import onnxruntime as ort
import scipy as sp
from matplotlib import pyplot as plt
from onnxruntime import InferenceSession
from scipy.special import softmax

from high_level.autotech_constant import (
    ANGLE_LOOKUP,
    MODEL_PATH,
    SPEED_LOOKUP,
    Temperature_steer,
    Temperature_vitesse,
)

STEER_LOOKUP_DEG = np.rad2deg(np.linspace(-0.4, 0.4, 16, dtype=np.float32))
SPEED_LOOKUP_MM_S = np.linspace(1.0, 9.0, 16, dtype=np.float32) * (1000.0 / 3.6)
# Webots training bins: steer in [-0.4, 0.4] rad, speed in [1.0, 9.0] km/h (or your config speed range)
WEBOTS_STEER_RAD = np.linspace(-0.4, 0.4, 16, dtype=np.float32)
WEBOTS_SPEED_KMH = np.linspace(1.0, 9.0, 16, dtype=np.float32)

class Driver:
    def __init__(self, context_size=0, horizontal_size=0):
        self.log = logging.getLogger(__name__)
        self.context_size = context_size
        self.horizontal_size = horizontal_size
        self._loaded = False
        self.ai_session: InferenceSession
        self.context: np.ndarray
        self.context_dict: np.Dict
        self.input_infos = None
        self.nb_inputs = 0
        self.last_angle = WEBOTS_SPEED_KMH
        self.last_speed = WEBOTS_STEER_RAD

        if self.log.isEnabledFor(logging.DEBUG):
            self.fig, self.ax = plt.subplots(4, 1, figsize=(10, 8))
            self.steering_bars = self.ax[0].bar(range(16), np.zeros(16), color="blue")
            self.steering_avg = [
                self.ax[0].plot(
                    [0, 0], [0, 1], color=(i / 3, 1 - i / 3, 0), label="Average"
                )[0]
                for i in range(4)
            ]
            self.ax[0].set_ylim(0, 1)  # Probabilities range from 0 to 1
            self.ax[0].set_title("Steering Action Probabilities")

            # Speed bars
            self.speed_bars = self.ax[1].bar(range(16), np.zeros(16), color="blue")
            self.speed_avg = self.ax[1].plot(
                [0, 0], [0, 1], color="red", label="Average"
            )[0]
            self.ax[1].set_ylim(0, 1)  # Probabilities range from 0 to 1
            self.ax[1].set_title("Speed Action Probabilities")

            # LiDAR img
            self.lidar_img = self.ax[2].imshow(
                np.zeros((128, 128)), cmap="gray", vmin=0, vmax=np.log(31)
            )
            self.ax[2].set_title("LiDAR Image")

            # Camera img
            self.camera_img = self.ax[3].imshow(
                np.zeros((128, 128, 3)), cmap="RdYlGn", vmin=-1, vmax=1
            )
            self.ax[3].set_title("Camera Image")

    def load_model(self, model: str):
        if self._loaded:
            return

        self.log.info("Loading AI model...")
        self.ai_session = ort.InferenceSession(MODEL_PATH + "/" + model)
        self.input_infos = self.ai_session.get_inputs()
        self.nb_inputs = len(self.input_infos)

        for i, info in enumerate(self.input_infos):
            self.log.info(
                f"Input {i}: name={info.name}, shape={info.shape}, type={info.type}"
            )

        first_shape = self.input_infos[0].shape

        if len(first_shape) == 2:
            self.model_kind = "lidar_only"
            self.context_size = 1
            self.horizontal_size = first_shape[-1]
        elif len(first_shape) == 4 and self.nb_inputs == 1:
            self.model_kind = "fused"
            self.context_size = first_shape[-2]
            self.horizontal_size = first_shape[-1]
            self.context = np.zeros(
                [2, self.context_size, self.horizontal_size], dtype=np.float32
            )
        elif len(first_shape) == 3:
            self.model_kind = "dict_inputs"
            self.context_size = int(first_shape[1])
            # Find horizontal resolution from Lidar/Cam input (or first input)
            self.horizontal_size = max(int(info.shape[-1]) for info in self.input_infos)
            self.context_dict = {
                info.name: np.zeros((1, int(info.shape[1]), int(info.shape[2])), dtype=np.float32)
                for info in self.input_infos
            }
        else:
            raise ValueError(
                f"Unsupported model format: nb_inputs={self.nb_inputs}, shape={first_shape}"
            )

        self._loaded = True
        self.log.info(f"AI model loaded with {self.nb_inputs} real input(s)")
        # self.context = np.zeros(
        #     [2, self.context_size, self.horizontal_size], dtype=np.float32
        # )

    def _resize_1d(self, arr: np.ndarray, target_width: int) -> np.ndarray:
        arr = np.asarray(arr, dtype=np.float32).reshape(-1)
        if arr.size == 0:
            return np.zeros(target_width, dtype=np.float32)
        if arr.size == target_width:
            return arr.astype(np.float32, copy=False)
        zoom_factor = target_width / arr.size
        return sp.ndimage.zoom(arr, zoom_factor, order=1).astype(np.float32, copy=False)

    def _resize_lidar_like_webots(self, lidar_data: np.ndarray) -> np.ndarray:
        """
        Like Webots:
        convert in meters, replace nan with 0, inf with 30, and resize to horizontal_size
        """
        # print("len(np.where(lidar_data == 0)[0]) =", len(np.where(lidar_data == 0)[0]))
        lidar_m = np.asarray(lidar_data, dtype=np.float32).reshape(-1)
        lidar_m = np.nan_to_num(lidar_m, nan=0.0, posinf=30.0, neginf=0.0)
        lidar_m = np.clip(lidar_m, 0.0, 30.0)
        lidar_m = self._resize_1d(lidar_m, self.horizontal_size)
        return lidar_m

    def ai(
        self, lidar_data: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        if getattr(self, "model_kind", None) == "dict_inputs":
            return self.ai_update_lidar_camera(lidar_data, camera_data)
        return self.ai_update_lidar(lidar_data)

    def get_nb_inputs(self) -> int:
        if not self._loaded:
            raise RuntimeError("Driver not initialized (AI model not loaded)")
        return self.nb_inputs

    def reset(self):
        self.last_angle = 0.0
        self.last_speed = 0.0
        if getattr(self, "model_kind", None) == "dict_inputs" and self.input_infos is not None:
            self.context_dict = {
                info.name: np.zeros((1, int(info.shape[1]), int(info.shape[2])), dtype=np.float32)
                for info in self.input_infos
            }
        else:
            self.context = np.zeros(
                [2, self.context_size, self.horizontal_size], dtype=np.float32
            )

    def omniscent(
        self, lidar_data_m: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        return self.ai_update_lidar_camera(lidar_data_m, camera_data)


    def simple_minded(
        self, lidar_data: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        # take the camera data for uniformity with the omniscent driver but we dont use it in this driver
        return self.farthest_distants(lidar_data)

    def ai_update_lidar_camera(
        self, lidar_data_m: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        if not self._loaded or self.input_infos is None:
            raise RuntimeError("Driver not initialized (AI model not loaded)")

        # 1. Clean & resize sensors
        lidar_data_m = self._resize_lidar_like_webots(lidar_data_m)[::-1]
        lidar_data_m[np.where(lidar_data_m == 0)] = 30.0
        camera_data = self._resize_camera_like_webots(camera_data)

        # 2. Update context & run ONNX depending on model_kind
        if self.model_kind == "dict_inputs":
            # Map possible ONNX input names to current 1D step arrays
            step_sources = {
                "Lidar": lidar_data_m,
                "lidar": lidar_data_m,
                "Cam": camera_data,
                "cam": camera_data,
                "Speed_Cmd": np.array([self.last_speed], dtype=np.float32),
                "Steer_Cmd": np.array([self.last_angle], dtype=np.float32),
            }
            for info in self.input_infos:
                name = info.name
                # Fallback if SB3 exported positional names like "input.1", "input.2"
                if name in step_sources:
                    raw_1d = step_sources[name]
                else:
                    raw_1d = lidar_data_m if info == self.input_infos[0] else camera_data
                new_frame = raw_1d.reshape(1, 1, -1)
                if self.context_size > 1:
                    self.context_dict[name] = np.concatenate(
                        [self.context_dict[name][:, 1:, :], new_frame], axis=1
                    )
                else:
                    self.context_dict[name] = new_frame

            vect = cast(
                np.ndarray,
                self.ai_session.run(None, self.context_dict)[0],
            )[0]

        elif self.model_kind == "fused":
            new_frame = np.stack([lidar_data_m, camera_data], axis=0)[:, None, :]  # (2,1,W)
            if self.context_size > 1:
                self.context = np.concatenate(
                    [self.context[:, 1:], new_frame], axis=1
                )
            else:
                self.context = new_frame
            input_name = self.input_infos[0].name
            vect = cast(
                np.ndarray,
                self.ai_session.run(None, {input_name: self.context[None]})[0],
            )[0]
        else:
            return self.ai_update_lidar(lidar_data_m)

        # 3. Decode 32 logits -> physical angle & speed via Temperature Softmax
        vect_dir, vect_prop = vect[:16], vect[16:]
        vect_dir = softmax(vect_dir / Temperature_steer)
        vect_prop = softmax(vect_prop / Temperature_vitesse)

        vitesse = float(sum(SPEED_LOOKUP * vect_prop))
        angle = float(sum(ANGLE_LOOKUP * vect_dir))

        # Store in Webots units (rad in [-0.4, 0.4] and km/h in [1.0, 9.0]) for Speed_Cmd / Steer_Cmd!
        self.last_angle = float(sum(WEBOTS_STEER_RAD * vect_dir))
        self.last_speed = float(sum(WEBOTS_SPEED_KMH * vect_prop))
        return angle, vitesse

    def ai_update_lidar(self, lidar_data) -> Tuple[float, float]:
        if not self._loaded:
            raise RuntimeError("Driver not initialized (AI model not loaded)")
        lidar_data = np.array(lidar_data, dtype=np.float32)
        # 2 vectors direction and speed. direction is between hard left at index 0 and hard right at index 1. speed is between min speed at index 0 and max speed at index 1

        vect = cast(
            np.ndarray, self.ai_session.run(None, {"input": lidar_data[None]})[0]
        )[0]

        vect_dir, vect_prop = vect[:16], vect[16:]  # split the vector in

        vect_dir = softmax(vect_dir / Temperature)  # probability distribution
        vect_prop = softmax(vect_prop)

        angle = sum(ANGLE_LOOKUP * vect_dir)  # weighted average of angles
        # weighted average of speeds
        vitesse = sum(SPEED_LOOKUP * vect_prop)
        return angle, vitesse

    def farthest_distants(self, lidar_data: np.ndarray) -> Tuple[float, float]:
        # Initialize variables
        lidar_data_mm = [0.0] * 360  # Assuming 360 degrees for the lidar data
        filter_size = 15
        max_value = 0
        max_index = 0
        closest_distance = float("inf")
        average_distance = 0
        valid_distance_count = 0

        # Translate lidar angles to table angles
        for angle in range(len(lidar_data_mm)):
            if 135 < angle < 225:
                lidar_data_mm[angle] = float("nan")
            else:
                lidar_data_mm[angle] = lidar_data[540 + (-angle * 4)]

        # Find the maximum value in the lidar data
        for i in range(-45, 45):
            sum_values = sum(
                lidar_data_mm[n] for n in range(i - filter_size, i + filter_size)
            )
            if sum_values > max_value:
                max_value = sum_values
                max_index = i
        # print("max_value =", max_value, "max_index =", max_index)

        # Calculate the average distance and find the closest object
        for i in range(-45, 45):
            current_distance = lidar_data_mm[i]
            if current_distance != 0:
                average_distance += current_distance
                valid_distance_count += 1
                if current_distance < closest_distance:
                    closest_distance = current_distance
        average_distance = (
            average_distance / valid_distance_count if valid_distance_count != 0 else 0
        )

        speed = average_distance * 0.002
        print("speed =", speed)
        speed = 2.0

        # Calculate the steering angle
        if speed >= 0.057:
            try:
                target_angle = (max_index / 180) * math.pi
                val = (1.35 * target_angle) / speed
                print("val =", val)
                steering_angle = (math.atan(val) / math.pi) * 180
            except Exception as e:
                steering_angle = 0.0
                print("error calculating angle:", e)
        else:
            steering_angle = 0.0

        return steering_angle, speed
