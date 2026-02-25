import logging
import threading
from enum import Enum

from actionneur_capteur import Camera, I2CArduino, Lidar, ToF
from high_level.autotech_constant import SOCKET_ADRESS

from .program import Program


class ProgramState(Enum):
    INITIALIZATION = 1
    RUNNING = 2
    STOPPED = 3


class Initialisation(Program):
    def __init__(self, server) -> None:
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.arduino_I2C_init = ProgramState.INITIALIZATION
        self.camera_init = ProgramState.INITIALIZATION
        self.lidar_init = ProgramState.INITIALIZATION
        self.tof_init = ProgramState.INITIALIZATION

        self.arduino_I2C = None
        self.camera = None
        self.lidar = None
        self.tof = None

        threading.Thread(target=self.init_camera, daemon=True).start()
        threading.Thread(target=self.init_lidar, daemon=True).start()
        threading.Thread(target=self.init_tof, daemon=True).start()
        threading.Thread(
            target=self.init_I2C_arduino,
            args=(server,),
            daemon=True,
        ).start()

    def init_I2C_arduino(self, server):
        try:
            self.arduino_I2C = I2CArduino(server)
            self.arduino_I2C_init = ProgramState.RUNNING
            self.log.info("I2C Arduino initialized successfully")
        except Exception as e:
            self.arduino_I2C_init = ProgramState.STOPPED
            self.log.error("I2C Arduino init error : " + str(e))

    def init_camera(self):
        try:
            self.camera = Camera()
            self.camera_init = ProgramState.RUNNING
            self.log.info("Camera initialized successfully")
        except Exception as e:
            self.camera_init = ProgramState.STOPPED
            self.log.error("Camera init error : " + str(e))

    def init_lidar(self):
        try:
            self.lidar = Lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
            self.lidar.stop()
            self.lidar.start_continuous(0, 1080)
            self.log.info("Lidar initialized successfully")
            self.lidar_init = ProgramState.RUNNING
        except Exception as e:
            self.lidar_init = ProgramState.STOPPED
            self.log.error("Lidar init error : " + str(e))

    def init_tof(self):
        try:
            self.tof = ToF()
            self.tof_init = ProgramState.RUNNING
            self.log.info("Camera initialized successfully")
        except Exception as e:
            self.tof_init = ProgramState.STOPPED
            self.log.error("Tof init error : " + str(e))

    def display(self) -> str:

        def state_to_text(state: ProgramState) -> str:
            match state:
                case ProgramState.INITIALIZATION:
                    return "(en cour)"
                case ProgramState.RUNNING:
                    return "ready."
                case ProgramState.STOPPED:
                    return "error"

            return "unknown"

        text = "\ncamera: "
        text += state_to_text(self.camera_init)
        text += "\n lidar: "
        text += state_to_text(self.lidar_init)
        text += "\n tof:"
        text += state_to_text(self.tof_init)
        text += "\n Arduino:"
        text += state_to_text(self.arduino_I2C_init)

        return text
