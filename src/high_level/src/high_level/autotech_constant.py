import os
import numpy as np
import logging

# Car control
MAX_IA_SPEED = 2 # maximum speed for ia
MIN_IA_SPEED = -2 # minimum speed for ia
MAX_CONTROL_SPEED = 2 # maximum speed for controling devices
MIN_CONTROL_SPEED = -2 # minimum speed for controlig devices
MAX_ANGLE = 18 # angle between the two extrem position


#I2C
I2C_NUMBER_DATA_RECEIVED = 3 # the number of info data sent by the arduino
I2C_SLEEP_RECEIVED = 0.1 # the time between two demand of info data to the arduino
I2C_SLEEP_ERROR_LOOP = 1 # In seconds its the time bettween two try of i2C if an error occurd
SLAVE_ADDRESS = 0x08 # Adresse of the arduino i2c port

#Remote control
PORT_REMOTE_CONTROL = 5556 # Port to send data for remote control on <IP>:PORT_REMOTE_CONTROL

#Camera
PORT_STREAMING_CAMERA = 8889 # adresse where to see the stream of the camera if activate is <IP>:PORT_STREAMIN_CAMERA/STREAM_PATH.jpeg
STREAM_PATH = "map"
SIZE_CAMERA_X = 1280
SIZE_CAMERA_Y = 720
FRAME_RATE = 30 # frame rate of the camera
CAMERA_QUALITY = 10 # the more the better but slow the speed of the stream (10 its passable)


#Car
CRASH_DIST = 110
REAR_BACKUP_DIST = 100  #mm Distance at which the car will NOT reverse due to the obstacle behind it

#Lidar
LIDAR_DATA_AMPLITUDE = 1
LIDAR_DATA_SIGMA = 45
LIDAR_DATA_OFFSET = 0.5

# Screen car
TEXT_HEIGHT = 11
TEXT_LEFT_OFFSET = 3 # Offset from the left of the screen to ensure no cuttoff



script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "model_CNN1D.onnx")  # Allows the model to be loaded from the same directory as the script regardless of the current working directory (aka where the script is run from)


SOCKET_ADRESS = {
    "IP": '192.168.0.10',
    "PORT": 10940
}

ANGLE_LOOKUP = np.linspace(-MAX_ANGLE, MAX_ANGLE, 16)
SPEED_LOOKUP = np.linspace(MIN_IA_SPEED, MAX_IA_SPEED, 16)

Temperature = 0.7 # Temperature parameter for softmax function, used to control the sharpness of the distribution resols around 1
# the higher the temperature the more unprobalbe actions become probable, the lower the temperature the more probable actions become probable.
# In our case Higher temperature means less agressive driving and lower temperature means more aggressive driving.


LOGGING_LEVEL = logging.DEBUG # can be either NOTSET, DEBUG, INFO, WARNING, ERROR, CRITICAL

#Startup
CAMERA_STREAM_ON_START = True #If True the camera stream will start at the launch of the car
BACKEND_ON_START = True #If True the backend will start at the launch of the car
LIDAR_STREAM_ON_START = True #If True the lidar stream will start at the launch of the car