from Lidar import Lidar
import time
from threading import Thread

#Pour le protocole I2C de communication entre la rasberie Pi et l'arduino
import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import numpy as np
import struct

SOCKET_ADRESS = {
    "IP": '192.168.0.10',
    "PORT": 10940
}


"""Initialize the Lidar sensor."""
try:
    lidar = Lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
    lidar.stop()
    lidar.startContinuous(0, 1080)
except Exception as e:
    raise
while(True):
    lidar_data = (lidar.rDistance[:1080]/1000)
    print("premiere valeur", lidar_data[1])
    print("deuxieme valeur", lidar_data[1075])