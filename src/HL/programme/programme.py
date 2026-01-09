from ..Autotech_constant import LOGGING_LEVEL
import logging
from typing import Optional

""" classe type pour tout les programme """
class Program:
    controls_car:bool # correspond si le programme cotronle la voiture ou non pour savoir si il faut arreter l'ancien programme qui controle la voiture
    running:bool # état de base du programme (au lancement de la voiture)
    vitesse_d:Optional[float] # target speed
    direction_d:Optional[float] # target direction

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
    