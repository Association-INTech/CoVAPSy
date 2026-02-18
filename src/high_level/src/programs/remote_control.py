from programs.program import Program
import struct
import socket
import threading
import logging
from high_level.autotech_constant import PORT_REMOTE_CONTROL


class RemoteControl(Program):
    """This program allows remote control of the car using UDP packets.
    You can take control with the script remote_control_controller.py"""

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.controls_car = True
        self.running = False
        self._target_speed = 0
        self._direction = 0

        # Initialization
        self.public = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.public.bind(("0.0.0.0", PORT_REMOTE_CONTROL))

        self.log.info("Remote control initialization finished")

    @property
    def target_speed(self) -> float:
        return self._target_speed

    @property
    def direction(self) -> float:
        return self._direction

    def car_controle(self, sock) -> None:
        """Starts control from the PC."""
        sock.settimeout(0.1)

        while self.running:
            try:
                data, ip = sock.recvfrom(1024)
                self._target_speed, self._direction = struct.unpack("ff", data)
            except socket.timeout:
                continue

    def start(self) -> None:
        self.running = True
        threading.Thread(
            target=self.car_controle, args=(self.public,), daemon=True
        ).start()

    def kill(self) -> None:
        """Exits the thread from its loop"""
        self.running = False
