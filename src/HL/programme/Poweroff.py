import subprocess
import os
from src.HL.programme.programme import Program

class Poweroff(Program):
    def __init__(self):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.name = "Power off"
        self.controls_car = False
        self.running = False

    def kill(self):
        pass

    def start(self):
        self.log("Power off started")
        subprocess.Popen(
                "sudo poweroff",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
    