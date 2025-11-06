import time
from ..src.HL.Lidar import Lidar


IP = '192.168.0.10'
PORT = 10940

if __name__ == '__main__':
    sensor = Lidar(IP, PORT)
    sensor.stop()
    # sensor.singleRead(0, 1080)
    time.sleep(2)

    sensor.startContinuous(0, 1080)
    sensor.startPlotter()