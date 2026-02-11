import board
import busio
from adafruit_vl53l1x import VL53L1X
import logging
import threading
import time

class ToF:
    """
    Class representing a Time of Flight (ToF) sensor.
    """

    def __init__(self):
        self.log = logging.getLogger(__name__)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = VL53L1X(i2c)
        self.distance = 0
        threading.Thread(target=self.get_tof_distance, daemon=True).start()
        
    def get_tof_distance(self):
        """
        Get the distance from the rear ToF sensor.
        """
        while True:
            try:
                self.distance = self.vl53.range
                time.sleep(0.01)  # Adjust the sleep time as needed
            except Exception as e:
                self.log.error(f"Error reading rear ToF sensor: {e}")
                return None
        
