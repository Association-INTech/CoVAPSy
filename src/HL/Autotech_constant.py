import os
import numpy as np

MAX_SOFT_SPEED = 1
MIN_SOFT_SPEED = 0.1
MAX_ANGLE = 14
CRASH_DIST = 110

script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "model.onnx")  # Allows the model to be loaded from the same directory as the script regardless of the current working directory (aka where the script is run from)

PWM_PROP = {
    "direction_prop": 1,
    "pwm_stop_prop": 7.37,
    "point_mort_prop": 0.5,
    "delta_pwm_max_prop": 1.1  # PWM at which the maximum speed is reached
}

PWM_DIR = {
    "direction": -1,  # 1 for angle_pwm_min to the left, -1 for angle_pwm_min to the right
    "angle_pwm_min": 6.91,
    "angle_pwm_max": 10.7,
    "angle_pwm_centre": 8.805
}

SOCKET_ADRESS = {
    "IP": '192.168.0.10',
    "PORT": 10940
}

ANGLE_LOOKUP = np.linspace(-MAX_ANGLE, MAX_ANGLE, 16)
SPEED_LOOKUP = np.linspace(MIN_SOFT_SPEED, MAX_SOFT_SPEED, 16)