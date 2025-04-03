import math
import scipy as sp
from scipy.special import softmax
import numpy as np
import onnxruntime as ort

from Autotech_constant import SPEED_LOOKUP, ANGLE_LOOKUP, MODEL_PATH



class Driver:
    def __init__(self, context_size=0, horizontal_size=0):
        self.ai_session = ort.InferenceSession(MODEL_PATH)
        self.context = np.zeros([2, context_size, horizontal_size], dtype=np.float32)

    def omniscent(self, lidar_data, camera_data):
        pass

    def ai(self, lidar_data, camera_data):
        return self.ai_update_lidar_camera(lidar_data, camera_data)

    def simple_minded(self, lidar_data):
        return self.farthest_distants(lidar_data)

    def ai_update_lidar_camera(self, lidar_data, camera_data):
        lidar_data = sp.ndimage.zoom(
            np.array(lidar_data, dtype=np.float32),
            128/len(lidar_data)
        )
        camera_data = sp.ndimage.zoom(
            np.array(camera_data, dtype=np.float32),
            128/len(camera_data)
        )

        self.context = np.concatenate([
            self.context[:, 1:],
            [lidar_data[None], camera_data[None]]
        ], axis=1)

        # 2 vectors direction and speed. direction is between hard left at index 0 and hard right at index 1. speed is between min speed at index 0 and max speed at index 1
        vect = self.ai_session.run(None, {'input': self.context[None]})[0][0]

        vect_dir, vect_prop = vect[:16], vect[16:]  # split the vector in 2
        vect_dir = softmax(vect_dir)  # distribution de probabilité
        vect_prop = softmax(vect_prop)

        print(" ".join([f"{x:.1f}" for x in vect_dir]))

        angle = sum(ANGLE_LOOKUP*vect_dir)  # moyenne pondérée des angles
        # moyenne pondérée des vitesses
        vitesse = sum(SPEED_LOOKUP*vect_prop)

        return angle, vitesse

    def ai_update_lidar(self, lidar_data):
        raise NotImplementedError("This method doesn't work anymore")
        lidar_data = np.array(lidar_data, dtype=np.float32)
        # 2 vectors direction and speed. direction is between hard left at index 0 and hard right at index 1. speed is between min speed at index 0 and max speed at index 1
        vect = self.ai_session.run(None, {'input': lidar_data[None]})[0][0]

        vect_dir, vect_prop = vect[:16], vect[16:]  # split the vector in 2
        vect_dir = softmax(vect_dir)  # distribution de probabilité
        vect_prop = softmax(vect_prop)

        angle = sum(ANGLE_LOOKUP*vect_dir)  # moyenne pondérée des angles
        # moyenne pondérée des vitesses
        vitesse = sum(SPEED_LOOKUP*vect_prop)

        return angle, vitesse



    def farthest_distants(self, lidar_data):
        # Initialize variables
        lidar_data_mm = [0] * 360  # Assuming 360 degrees for the lidar data
        filter_size = 15
        max_value = 0
        max_index = 0
        closest_distance = float('inf')
        average_distance = 0
        valid_distance_count = 0

        # Translate lidar angles to table angles
        for angle in range(len(lidar_data_mm)):
            if 135 < angle < 225:
                lidar_data_mm[angle] = float('nan')
            else:
                lidar_data_mm[angle] = lidar_data[540 + (-angle * 4)]

        # Find the maximum value in the lidar data
        for i in range(-45, 45):
            sum_values = sum(lidar_data_mm[n] for n in range(i - filter_size, i + filter_size))
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
        average_distance = average_distance / valid_distance_count if valid_distance_count != 0 else 0

        speed = average_distance * 0.002
        print("speed =", speed)
        speed = 2

        # Calculate the steering angle
        if speed >= 0.057:
            try:
                target_angle = (max_index / 180) * math.pi
                val = (1.35 * target_angle) / speed
                print("val =", val)
                steering_angle = (math.atan(val) / math.pi) * 180
            except Exception as e:
                steering_angle = 0
                print("error calculating angle:", e)
        else:
            steering_angle = 0

        return steering_angle, speed
