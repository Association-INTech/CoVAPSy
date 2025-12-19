from ..Autotech_constant import LOGGING_LEVEL
import logging
from typing import Optional

""" classe type pour tout les programme """
class Program:
    controls_car:bool 
    running:bool
    vitesse_d:Optional[float] # change me to target speed
    direction_d:Optional[float]

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
    