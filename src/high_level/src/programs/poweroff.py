import subprocess
import os
from programs.program import Program
import logging


class Poweroff(Program):
    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.controls_car = False
        self.running = False

    def kill(self) -> None:
        pass

    def start(self) -> None:
        self.log.info("Power off started")
        subprocess.Popen(
            "sudo poweroff",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
