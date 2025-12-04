from programme import Program
import struct
import socket
import threading

class RemoteControl(Program):
    def __init__(self):
        super().__init__()
        self.name = "Remote Control"
        self.controls_car = True
        self.running = False
        self.vitesse_d = 0
        self.direction = 0
    
        #initialisation
        self.public = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.public.bind(("0.0.0.0", 5556))

    def car_controle(self,sock):
        """ on regarde si il s'agit de lappelle pour le control interne 
        (is_private) ou si on veux prendre le controle depuis le pc."""
        sock.settimeout(0.1)

        while self.running:
            try:
                data, ip = sock.recvfrom(1024)
                self.vitesse_d, self.direction = struct.unpack("ff", data)
                self.last_cmd_time = time.time()
            except socket.timeout:
                continue

    def start(self):
        self.running = True
        threading.Thread(target=self.car_controle, args=(self.public,), daemon=True).start()
    
    def kill(self):
        self.running = False