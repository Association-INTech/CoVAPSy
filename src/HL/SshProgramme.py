from programme import Program
from get_ip import get_ip, check_ssh_connections

class SshProgramme(Program):
    def __init__(self):
        self.name = "Ssh to:" + get_ip()
        self.running = False
        self.controls_car = True

        self.vitesse_d = 0
        self.direction = 0
    
    def display(self):
        text = self.name
        if check_ssh_connections():
            text+= "\n connecté"
        if self.running:
            text+= "\n Voiture en stand by"
        return text