import os
import numpy as np

MAX_SOFT_SPEED = 6000 # en milimetre par secondes
MIN_SOFT_SPEED = -4000
MAX_ANGLE = 30
CRASH_DIST = 110
REAR_BACKUP_DIST = 100  #mm Distance at which the car will NOT reverse due to the obstacle behind it
LIDAR_DATA_AMPLITUDE = 1
LIDAR_DATA_SIGMA = 45
LIDAR_DATA_OFFSET = 0.5

script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "model_CNN1D.onnx")  # Allows the model to be loaded from the same directory as the script regardless of the current working directory (aka where the script is run from)


SOCKET_ADRESS = {
    "IP": '192.168.0.10',
    "PORT": 10940
}

ANGLE_LOOKUP = np.linspace(-MAX_ANGLE, MAX_ANGLE, 16)
SPEED_LOOKUP = np.linspace(MIN_SOFT_SPEED, MAX_SOFT_SPEED, 16)

Temperature = 0.7 # Temperature parameter for softmax function, used to control the sharpness of the distribution resols around 1
# the higher the temperature the more unprobalbe actions become probable, the lower the temperature the more probable actions become probable.
# In our case Higher temperature means less agressive driving and lower temperature means more aggressive driving.