from programme import Program
from get_ip import get_ip

class SshProgramme(Program):
    def __init__(self):
        self.name = "Ssh to:" + get_ip()
        self.running = False
        self.controls_car = False
