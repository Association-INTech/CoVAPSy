import time
from ..src.HL.HokuyoReader import HokuyoReader


IP = '192.168.0.10'
PORT = 10940

if __name__ == '__main__':
    sensor = HokuyoReader(IP, PORT)
    sensor.stop()
    # sensor.singleRead(0, 1080)
    time.sleep(2)

    sensor.startContinuous(0, 1080)
    sensor.startPlotter()