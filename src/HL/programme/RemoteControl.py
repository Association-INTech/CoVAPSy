from src.HL.programme.programme import Program
import struct
import socket
import threading
import time
import logging 
from src.HL.Autotech_constant import PORT_REMOTE_CONTROL
class RemoteControl(Program):
    """ ce programme permet de prendre le control de la voiture à distance en utilsant des packet udp
     on peut prendre le controle avec le script remote_controle_controller.py"""
    def __init__(self):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.controls_car = True
        self.running = False
        self.vitesse_d = 0
        self.direction_d = 0
    
        #initialisation
        self.public = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.public.bind(("0.0.0.0", PORT_REMOTE_CONTROL))

        self.log.info("Initialisation de Remote control fini")

    def car_controle(self,sock):
        """lance le controle depuis le pc."""
        sock.settimeout(0.1)

        while self.running:
            try:
                data, ip = sock.recvfrom(1024)
                self.vitesse_d, self.direction_d = struct.unpack("ff", data)
            except socket.timeout:
                continue

    def start(self):
        self.running = True
        threading.Thread(target=self.car_controle, args=(self.public,), daemon=True).start()
    
    def kill(self):
        """fait sortir le thread de sa boucle"""
        self.running = False