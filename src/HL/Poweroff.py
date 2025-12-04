import subprocess
from programme import Program

class Poweroff(Program):
    def __init__(self):
        super().__init__()
        self.name = "Power off"
        self.controls_car = False
        self.running = False

    def kill():
        pass

    def start():
        subprocess.Popen(
                "sudo poweroff",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
    