from src.HL.programme.programme import Program
from src.HL.programme.scripts.get_ip import check_ssh_connections
import socket
import time

class SshProgramme(Program):
    """Montre le menu SSH de la voiture et force vitesse/direction à 0"""

    def __init__(self):
        super().__init__()
        self.running = True
        self.controls_car = True

        self.target_speed = 0
        self.direction = 0

        # Cache IP
        self.ip = None
        self._last_ip_check = 0
        self._ip_refresh_interval = 1.0  # secondes

    def start(self):
        self.running = True

    def stop(self):
        self.running = False

    def _update_ip_if_needed(self):
        now = time.monotonic()
        if now - self._last_ip_check >= self._ip_refresh_interval:
            self.ip = self.get_local_ip()
            self._last_ip_check = now

    @staticmethod
    def get_local_ip():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return None

    def display(self):
        self._update_ip_if_needed()

        text = f"Ssh to: {self.ip or 'non connecté'}"
        if check_ssh_connections():
            text += "\n connecté"
        if self.running:
            text += "\n Voiture en stand by"
        return text
