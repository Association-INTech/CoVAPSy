import time
import board
import busio
from adafruit_vl53l0x import VL53L0X
import logging

class ToF:
    """
    Class representing a Time of Flight (ToF) sensor.
    """

    def __init__(self):
        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = VL53L0X(i2c)
        
    def get_rear_distance(self):
        """
        Get the distance from the rear ToF sensor.
        """
        try:
            distance = self.vl53.range
            return distance
        except Exception as e:
            logging.error(f"Error reading rear ToF sensor: {e}")
            return None
        
