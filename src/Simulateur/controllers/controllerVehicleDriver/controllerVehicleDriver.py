import numpy as np
import psutil
import time
import os
import re 
import sys
import logging

# add src/Simulateur to sys.path
path = __file__.rsplit('/', 3)[0]
sys.path.insert(0, path)

from config import *
from vehicle import Driver
from enum import Enum,auto

def too_close(lidar,dir):
    # if dir==True: we're near the right wall
    R=0.83
    l=len(lidar)
    straight=lidar[l//2]
    if dir:
        nearest=min(lidar[l//2:])
    else:
        nearest=min(lidar[:l//2])
    theta=np.arccos(nearest/straight)
    L=R*(1-np.sin(theta))
    return nearest < L

class State(Enum):
    AI=auto()
    BACK=auto()

class VehicleDriver(Driver):
    """
    This class is a subclass of the Driver class and is used to control the vehicle.
    It basically receives instructions from the controllerWorldSupervisor and follows them.
    """

    def __init__(self):
        super().__init__()
        self.state=State.AI


        basicTimeStep = int(self.getBasicTimeStep())
        self.sensorTime = basicTimeStep // 4

        self.v_min = 1 
        self.v_max = 9 

        self.i = int(self.getName().split("_")[-1])
        # Lidar
        self.lidar = self.getDevice("Hokuyo")
        self.lidar.enable(self.sensorTime)
        self.lidar.enablePointCloud()

        # Camera
        self.camera = self.getDevice("RASPI_Camera_V2")
        self.camera.enable(self.sensorTime)

        # Checkpoint sensor
        self.touch_sensor = self.getDevice("touch_sensor")
        self.touch_sensor.enable(self.sensorTime)

        # Communication

        proc = psutil.Process(os.getpid()) #current
        parent = proc.parent() #parent
        grandparent = parent.parent() if parent else None #grandparent
        pppid = str(grandparent.pid)


        self.simulation_rank = int(
            re.search(
                pppid + r" (\d+)",
                open("/tmp/autotech/simulationranks", "r").read(),
                re.MULTILINE
            ).group(1)
        )

        self.handler = logging.FileHandler(f"/tmp/autotech/Voiture_{self.simulation_rank}_{self.i}.log")
        self.handler.setFormatter(FORMATTER)
        self.log = logging.getLogger("CLIENT")
        self.log.setLevel(level=LOG_LEVEL)
        self.log.addHandler(self.handler)

        self.log.debug("Connection to the server")
        self.fifo_r = open(f"/tmp/autotech/serverto{self.simulation_rank}_{self.i}.pipe", "rb")
        self.log.debug("Connection with the supervisor")
        self.fifo_w = open(f"/tmp/autotech/{self.simulation_rank}_{self.i}tosupervisor.pipe", "wb")




    #Vérification de l"état de la voiture
    def observe(self):
        try:
            sensor_data = [np.array(self.touch_sensor.getValue(), dtype=np.float32)]

            lidar_data = np.array(self.lidar.getRangeImage(), dtype=np.float32)

            camera_data = np.array(self.camera.getImageArray(), dtype=np.float32)
            # shape = (1080, 1, 3)
            camera_data = camera_data.transpose(1, 2, 0)[0]
            # shape = (3, 1080)
            color = np.argmax(camera_data, axis=0)
            camera_data = (
                (color == 0).astype(np.float32)*-1 +
                (color == 1).astype(np.float32)*1 +
                (color == 2).astype(np.float32)*0
            )
            # red   -> -1
            # green -> 1
            # blue  -> 0

            return (
                sensor_data,
                lidar_data,
                camera_data
            )
        except:
            #En cas de non retour lidar
            return (
                np.array(self.touch_sensor.getValue(), dtype=np.float32),
                np.zeros(self.lidar.getNumberOfPoints(), dtype=np.float32)
            )

    #Fonction step de l"environnement GYM
    def step(self):
        match self.state:
            case State.AI:
                self.ai()
            case State.BACK:
                self.back()

        return super().step()
        
         

    def ai(self):
        # sends observation to the supervisor
        self.emitter.send(np.concatenate(self.observe()).tobytes())
        
        # First to be executed
        
        self.log.info("Starting step")
        obs = self.observe()
        self.log.info(f"Observe {obs=}")
        self.fifo_w.write(obs.tobytes())
        self.fifo_w.flush() 
        
        self.log.debug("Trying to read action from the server")    
        action = np.frombuffer(self.fifo_r.read(np.dtype(np.int64).itemsize * 2), dtype=np.int64)
        self.log.info(f"received {action=}")

        # Simulation step

        action_steering = np.linspace(-.4, .4, n_actions_steering, dtype=np.float32)[action[0], None]
        action_speed = np.linspace(self.v_min, self.v_max, n_actions_speed, dtype=np.float32)[action[1], None]

        cur_angle = self.getSteeringAngle()
        dt = self.getBasicTimeStep()
        omega = 20 # rad/s (max angular speed of the steering servo)

        action_steering = cur_angle + np.clip(action_steering - cur_angle, -omega * dt, omega * dt)

        self.setSteeringAngle(action_steering)
        self.setCruisingSpeed(action_speed)

        if self.touch_sensor.getValue():
            self.state=State.BACK

    
    def back(self):
        #si mur de "dir": braquer à "dir"" et reculer jusqu'à pouvoir réavancer (distance au mur à vérif)
        lidar,cam=self.observe()[1:]
        S=sum(cam)
        dir = S>0
        if dir:
            self.setSteeringAngle(0.33)
            if too_close(lidar,dir):
                self.setCruisingSpeed(-2)
            else: self.state=State.AI
        else:
            self.setSteeringAngle(-0.33)
            if too_close(lidar,dir):
                self.setCruisingSpeed(-2)
            else: self.state=State.AI

    def run(self):
        # this call is just there to make sure at least one step
        # is done in the entire simulation before we call lidar.getRangeImage()
        # otherwise it will crash the controller with the message:
        # WARNING: 'controllerVehicleDriver' controller crashed.
        # WARNING: controllerVehicleDriver: The process crashed some time after starting successfully.
        super().step()
        while self.step() != -1:
            pass


#----------------Programme principal--------------------
def main():
    driver = VehicleDriver()
    driver.log.info("Starting the vehicle driver\n")
    driver.run()


if __name__ == "__main__":
    print("Starting the vehicle driver")
    main()
