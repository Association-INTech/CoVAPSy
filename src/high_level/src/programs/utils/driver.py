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
    Temperature,
)


class Driver:
    def __init__(self, context_size=0, horizontal_size=0):
        self.log = logging.getLogger(__name__)
        self.context_size = context_size
        self.horizontal_size = horizontal_size
        self._loaded = False
        self.ai_session: InferenceSession
        self.context: np.ndarray
        self.input_infos = None
        self.nb_inputs = 0

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
        elif self.nb_inputs == 2:
            self.model_kind = "two_inputs"
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

    def _resize_lidar_like_webots(self, lidar_data_mm: np.ndarray) -> np.ndarray:
        """
        Like Webots:
        convert in meters, replace nan with 0, inf with 30, and resize to horizontal_size
        """
        lidar_m = np.asarray(lidar_data_mm, dtype=np.float32).reshape(-1) / 1000.0
        lidar_m = np.nan_to_num(lidar_m, nan=0.0, posinf=30.0, neginf=0.0)
        lidar_m = np.clip(lidar_m, 0.0, 30.0)
        lidar_m = self._resize_1d(lidar_m, self.horizontal_size)
        return lidar_m

    def _resize_camera_like_webots(self, camera_data: np.ndarray) -> np.ndarray:
        """
        Like Webots:
        - float32
        - nan -> 0, +inf -> 30
        - resize into horizontal_size
        """
        cam = np.asarray(camera_data, dtype=np.float32).reshape(-1)
        cam = np.nan_to_num(cam, nan=0.0, posinf=30.0, neginf=-30.0)
        cam = self._resize_1d(cam, self.horizontal_size)
        return cam

    def get_nb_inputs(self) -> int:
        if not self._loaded:
            raise RuntimeError("Driver not initialized (AI model not loaded)")
        return self.nb_inputs

    def reset(self):
        self.context = np.zeros(
            [2, self.context_size, self.horizontal_size], dtype=np.float32
        )
        pass

    def omniscent(
        self, lidar_data: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        return self.ai_update_lidar_camera(lidar_data, camera_data)

    def ai(
        self, lidar_data: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        # take the camera data for uniformity with the omniscent driver but we dont use it in this driver
        return self.ai_update_lidar(lidar_data)

    def simple_minded(
        self, lidar_data: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        # take the camera data for uniformity with the omniscent driver but we dont use it in this driver
        return self.farthest_distants(lidar_data)

    def ai_update_lidar_camera(
        self, lidar_data_mm: np.ndarray, camera_data: np.ndarray
    ) -> Tuple[float, float]:
        if not self._loaded or self.input_infos is None:
            raise RuntimeError("Driver not initialized (AI model not loaded)")

        target_width = self.horizontal_size
        lidar_data = self._resize_lidar_like_webots(lidar_data_mm)
        camera_data = self._resize_camera_like_webots(camera_data)

        new_frame = np.stack([lidar_data, camera_data], axis=0)[:, None, :]  # (2,1,W)

        if self.context_size > 1:
            self.context = np.concatenate([self.context[:, 1:], new_frame], axis=1)
        else:
            self.context = new_frame

        input_name = self.input_infos[0].name
        vect = cast(
            np.ndarray,
            self.ai_session.run(None, {input_name: self.context[None]})[0],
        )[0]

        vect_dir, vect_prop = vect[:16], vect[16:]
        vect_dir = softmax(vect_dir)
        vect_prop = softmax(vect_prop)

        angle = sum(ANGLE_LOOKUP * vect_dir)
        vitesse = sum(SPEED_LOOKUP * vect_prop)
        best_idx = int(np.argmax(vect_dir))
        self.log.info(f"best_idx={best_idx}, angle={angle:.2f}")
        return angle, vitesse

    def ai_update_lidar(self, lidar_data) -> Tuple[float, float]:
        if not self._loaded:
            raise RuntimeError("Driver not initialized (AI model not loaded)")
        lidar_data = np.array(lidar_data, dtype=np.float32) * 1.6
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
        print("max_value =", max_value, "max_index =", max_index)

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
