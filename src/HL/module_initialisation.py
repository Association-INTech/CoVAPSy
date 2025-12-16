
from programme import Program


class Initialisation(Program):
    def __init__(self, camera, lidar, tof):
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
    
    def init_lidar(self,lidar):
        try:
            self.lidar = lidar()
            self.lidar_init = 1
        except Exception as e:
            self.lidar_init = 2
            self.error += str(e)

    def init_tof(self,tof):
        try:
            self.tof = tof()
            self.tof_init = 1
        except Exception as e:
            self.tof_init = 2
            self.error += str(e)

    def display(self):
        text = self.name
        
        text+= "camera: "
        if self.camera_init == 0:
            text += "(en cour)"
        elif self.camera_init == 1:
            text += "près."
        elif self.camera_init == 2:
            text += "error"

        text+= "\nlidar: "
        if self.lidar_init == 0:
            text += "(en cour)"
        elif self.lidar_init == 1:
            text += "près."
        elif self.lidar_init == 2:
            text += "error"

        text+= "\ntof:"
        if self.tof_init == 0:
            text += "(en cour)"
        elif self.tof_init == 1:
            text += "près."
        elif self.tof_init == 2:
            text += "error"
        
        return text