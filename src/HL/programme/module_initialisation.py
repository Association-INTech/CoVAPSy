
from src.HL.programme.programme import Program
import threading
from ..Autotech_constant import SOCKET_ADRESS
import logging as log
class Initialisation(Program):
    def __init__(self, camera, lidar, tof):
        super().__init__()
        self.name = "Initialisation:"
        self.camera = None
        self.lidar = None
        self.tof = None
        self.camera_init = 0
        self.lidar_init = 0
        self.tof_init = 0
        self.error = ""
        threading.Thread(target=self.init_camera, args=(camera,), daemon=True).start()
        threading.Thread(target=self.init_lidar, args=(lidar,), daemon=True).start()
        threading.Thread(target=self.init_tof, args=(tof,), daemon=True).start()


    def init_camera(self,camera):
        try:
            self.camera = camera()
            self.camera_init = 1
        except Exception as e:
            self.camera_init = 2
            self.error += str(e)
            print("-------------------------------------------")
            print(self.error)
    
    def init_lidar(self,lidar):
        try:
            self.lidar = lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
            self.lidar.stop()
            self.lidar.startContinuous(0, 1080)
            log.info("Lidar initialized successfully")
            self.lidar_init = 1
        except Exception as e:
            self.lidar_init = 2
            self.error += str(e)
            print("-------------------------------------------")
            print(self.error)

    def init_tof(self,tof):
        try:
            self.tof = tof()
            self.tof_init = 1
        except Exception as e:
            self.tof_init = 2
            self.error += str(e)
            print("-------------------------------------------")
            print(self.error)

    def display(self):
        text = self.name
        
        text+= "\n camera: "
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
        
        return text