from Autotech_constant import LOGGING_LEVEL
import logging
from typing import Optional

class Program:
    name:str
    controls_car:bool
    running:bool
    vitesse_d:Optional[float]
    direction:Optional[float]

    def __init__(self):
        self.logger = logging.getLogger()
        self.logger.setLevel(LOGGING_LEVEL)

    def kill(self):
        pass

    def start(self):
        pass

    def display(self):
        if (self.running):
            return self.name + "\n" + "(running)"
        else:
            return self.name + "\n"
    