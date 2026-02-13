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
        self.vl53.stop_ranging()  # Ensure sensor is stopped before starting
        time.sleep(0.1)  # Short delay to ensure sensor is ready
        self.vl53.start_ranging()
        while True:
            try:
                if self.vl53.data_ready:
                    self.distance = self.vl53.distance if self.vl53.distance is not None else 0 # en cm
                    self.vl53.clear_interrupt()
                time.sleep(0.05)
            except Exception as e:
                self.log.error(f"Error reading rear ToF sensor: {e}")
                return None
        
