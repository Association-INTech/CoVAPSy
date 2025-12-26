import os
import numpy as np

# controle de la voiture
MAX_IA_SPEED = 2 # maximum speed for ia
MIN_IA_SPEED = -2 # minimum speed for ia
MAX_CONTROL_SPEED = 2 # maximum speed for controling devices
MIN_CONTROL_SPEED = -2 # minimum speed for controlig devices
MAX_ANGLE = 18 # angle between the two extrem position


#I2C
I2C_NUMBER_DATA_RECEIVED = 3 # the number of info data sent by the arduino
I2C_SLEEP_RECEIVED = 0.1 # the time between two demand of info data to the arduino
I2C_SLEEP_ERROR_LOOP = 1 # In seconds its the time bettween two try of i2C if an error occurd

#Remote control
PORT_REMOTE_CONTROL = 5556

CRASH_DIST = 110
REAR_BACKUP_DIST = 100  #mm Distance at which the car will NOT reverse due to the obstacle behind it
LIDAR_DATA_AMPLITUDE = 1
LIDAR_DATA_SIGMA = 45
LIDAR_DATA_OFFSET = 0.5

script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "model_CNN1D.onnx")  # Allows the model to be loaded from the same directory as the script regardless of the current working directory (aka where the script is run from)

SLAVE_ADDRESS = 0x08
SOCKET_ADRESS = {
    "IP": '192.168.0.10',
    "PORT": 10940
}

ANGLE_LOOKUP = np.linspace(-MAX_ANGLE, MAX_ANGLE, 16)
SPEED_LOOKUP = np.linspace(MIN_SOFT_SPEED, MAX_SOFT_SPEED, 16)

Temperature = 0.7 # Temperature parameter for softmax function, used to control the sharpness of the distribution resols around 1
# the higher the temperature the more unprobalbe actions become probable, the lower the temperature the more probable actions become probable.
# In our case Higher temperature means less agressive driving and lower temperature means more aggressive driving.

import logging
LOGGING_LEVEL = logging.DEBUG # can be either NOTSET, DEBUG, INFO, WARNING, ERROR, CRITICAL