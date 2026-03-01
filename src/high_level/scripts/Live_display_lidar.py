import time
from driver.lidar import Lidar


IP = "192.168.0.20"
PORT = 10940

if __name__ == "__main__":
    sensor = Lidar(IP, PORT)
    sensor.stop()
    # sensor.singleRead(0, 1080)
    time.sleep(2)

    sensor.start_continuous(0, 1080)
    sensor.start_plotter()
