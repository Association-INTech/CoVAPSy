from high_level.autotech_constant import LOGGING_LEVEL
import logging
from typing import Optional

""" Base class for all programs """
class Program:
    controls_car:bool # whether the program controls the car or not to know if we need to stop the old program that controls the car
    running:bool # base state of the program (on car startup)
    target_speed:Optional[float] # target speed
    direction:Optional[float] # target direction

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(LOGGING_LEVEL)

    def kill(self):
        pass

    def start(self):
        pass

    def display(self):
        name = self.__class__.__name__
        if (self.running):
            return f"{name} \n (running)"
        else:
            return name
    