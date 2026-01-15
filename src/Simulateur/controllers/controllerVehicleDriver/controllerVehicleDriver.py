import numpy as np
import time

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
        self.receiver = self.getDevice("TT02_receiver")
        self.receiver.enable(self.sensorTime)
        self.receiver.setChannel(2 * self.i) # corwe ponds the the supervisor's emitter channel
        self.emitter = self.getDevice("TT02_emitter")
        self.emitter.setChannel(2 * self.i + 1) # corresponds the the supervisor's receiver channel

        # Last data received from the supervisor (steering angle)
        self.last_data = np.zeros(2, dtype=np.float32)

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

        if self.receiver.getQueueLength() > 0:
            while self.receiver.getQueueLength() > 1:
                self.receiver.nextPacket()
            self.last_data = np.frombuffer(self.receiver.getBytes(), dtype=np.float32)

        action_steering, action_speed = self.last_data

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
    driver.run()


if __name__ == "__main__":
    print("Starting the vehicle driver")
    main()
