from src.HL.programme.programme import Program
from src.HL.programme.scripts.get_ip import get_ip, check_ssh_connections
import logging
class SshProgramme(Program):
    """montre le menu ssh de la voiture et si séléctionner comme programme force la vitesse et la direction à 0"""
    def __init__(self):
        super().__init__()
        self.ip = get_ip()
        self.running = True
        self.controls_car = True

        self.vitesse_d = 0
        self.direction_d = 0
    

    def start(self):
        self.running = True
    
    def stop(self):
        self.running = False

    def display(self):
        text = "Ssh to:" + self.ip
        if check_ssh_connections():
            text+= "\n connecté"
        if self.running:
            text+= "\n Voiture en stand by"
        return text