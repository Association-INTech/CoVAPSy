from _socket import _RetAddress


import socket
import time

from programs.program import Program
from programs.utils.ssh import check_ssh_connections


class SshProgramme(Program):
    """Give information about SSH connections and IP address and if the car is in standby mode (when this program is running)"""

    def __init__(self) -> None:
        super().__init__()
        self.running = True
        self.controls_car = True

        # Cache IP
        self.ip = None
        self._last_ip_check = 0
        self._ip_refresh_interval = 1.0  # secondes

    @property
    def target_speed(self) -> float:
        return 0.0

    @property
    def direction(self) -> float:
        return 0.0

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def _update_ip_if_needed(self) -> None:
        now = time.monotonic()
        if now - self._last_ip_check >= self._ip_refresh_interval:
            self.ip = self.get_local_ip()
            self._last_ip_check = now

    @staticmethod
    def get_local_ip() -> _RetAddress | None:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return None

    def display(self) -> str:
        self._update_ip_if_needed()

        text = f"Ssh to: {self.ip or 'not connected'}"
        if check_ssh_connections():
            text += "\n connected"
        if self.running:
            text += "\n Car in standby"
        return text
