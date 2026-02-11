import board
import busio
from adafruit_vl53l1x import VL53L1X
import logging

class ToF:
    """
    Class representing a Time of Flight (ToF) sensor.
    """

    def __init__(self):
        self.log = logging.getLogger(__name__)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = VL53L1X(i2c)
        
    def get_tof_distance(self):
        """
        Get the distance from the rear ToF sensor.
        """
        try:
            distance = self.vl53.range
            return distance
        except Exception as e:
            self.log.error(f"Error reading rear ToF sensor: {e}")
            return None
        
