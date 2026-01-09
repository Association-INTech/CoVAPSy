
from src.HL.programme.programme import Program
import threading
from ..Autotech_constant import SOCKET_ADRESS
import logging

class Initialisation(Program):
    def __init__(self,server, camera, lidar, tof, I2C):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.arduino_I2C = None
        self.camera = None
        self.lidar = None
        self.tof = None
        self.arduino_I2C_init = 0
        self.camera_init = 0
        self.lidar_init = 0
        self.tof_init = 0

        threading.Thread(target=self.init_camera, args=(camera,), daemon=True).start()
        threading.Thread(target=self.init_lidar, args=(lidar,), daemon=True).start()
        threading.Thread(target=self.init_tof, args=(tof,), daemon=True).start()
        threading.Thread(target=self.init_I2C_arduino, args=(I2C,server,), daemon=True).start()

    def init_I2C_arduino(self,I2C,server):
        try:
            self.arduino_I2C = I2C(server)
            self.arduino_I2C_init = 1
            self.log.info("I2C Arduino initialized successfully")
        except Exception as e:
            self.arduino_I2C_init = 2
            self.log.error("I2C Arduino init error : " + str(e))
    

    def init_camera(self,camera):
        try:
            self.camera = camera()
            self.camera_init = 1
            self.log.info("Camera initialized successfully")
        except Exception as e:
            self.camera_init = 2
            self.log.error("Camera init error : " + str(e))
    
    def init_lidar(self,lidar):
        try:
            self.lidar = lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
            self.lidar.stop()
            self.lidar.startContinuous(0, 1080)
            self.log.info("Lidar initialized successfully")
            self.lidar_init = 1
        except Exception as e:
            self.lidar_init = 2
            self.log.error("Lidar init error : " + str(e))

    def init_tof(self,tof):
        try:
            self.tof = tof()
            self.tof_init = 1
            self.log.info("Camera initialized successfully")
        except Exception as e:
            self.tof_init = 2
            self.log.error("Tof init error : " + str(e))

    def display(self):

        text = "\ncamera: "
        if self.camera_init == 0:
            text += "(en cour)"
        elif self.camera_init == 1:
            text += "ready."
        elif self.camera_init == 2:
            text += "error"

        text+= "\n lidar: "
        if self.lidar_init == 0:
            text += "(en cour)"
        elif self.lidar_init == 1:
            text += "ready."
        elif self.lidar_init == 2:
            text += "error"

        text+= "\n tof:"
        if self.tof_init == 0:
            text += "(en cour)"
        elif self.tof_init == 1:
            text += "ready."
        elif self.tof_init == 2:
            text += "error"

        text+= "\n I2C Arduino:"
        if self.arduino_I2C_init == 0:
            text += "(en cour)"
        elif self.arduino_I2C_init == 1:
            text += "ready."
        elif self.arduino_I2C_init == 2:
            text += "error"
        
        return text