import re
from typing import *
import numpy as np
import gymnasium as gym

from checkpointmanager import CheckpointManager, checkpoints

from controller import Supervisor
supervisor = Supervisor()
import torch.nn as nn

import psutil
import os
import sys

# this is necessary because we are not in a package context
# so we cannot do from ..config import *
# SO EVEN IF IT LOOKS UGLY, WE HAVE TO DO THIS
script_dir = os.path.dirname(os.path.abspath(__file__))
controllers_path = os.path.join(script_dir, '../..')
sys.path.append(controllers_path)

from config import *

checkpoint_positions = [
    [-0.314494, -2.47211, 0.0391],
    [1.11162, -2.56708, 0.0391],
    [2.54552, -2.27446, 0.0391],
    [3.58779, -1.38814, 0.0391],
    [3.58016, -0.0800134, 0.0391],
    [3.23981, 1.26309, 0.0391],
    [2.8261, 1.99783, 0.0391],
    [3.18851, 2.71151, 0.0391],
    [3.6475, 4.09688, 0.0391],
    [3.1775, 4.44688, 0.0391],
    [2.58692, 4.5394, 0.0391],
    [1.52457, 4.3991, 0.0391],
    [0.659969, 3.57074, 0.0391],
    [0.000799585, 2.90417, 0.0391],
    [0.0727115, 1.81299, 0.0391],
    [0.788956, 1.22248, 0.0391],
    [1.24749, 0.288391, 0.0391],
    [0.88749, -0.281609, 0.0391],
    [0.0789172, -0.557653, 0.0391],
    [-0.832859, -0.484867, 0.0391],
    [-1.79723, 0.408769, 0.0391],
    [-1.7446, 1.3386, 0.0391],
    [-1.92104, 2.72452, 0.0391],
    [-2.96264, 2.96666, 0.0391],
    [-4.19027, 2.74619, 0.0391],
    [-4.34725, 1.7503, 0.0391],
    [-4.26858, 0.259482, 0.0391],
    [-4.20936, -1.06968, 0.0391],
    [-4.0021, -2.35518, 0.0391],
    [-2.89371, -2.49154, 0.0391],
    [-2.01029, -2.51669, 0.0391],
]

# webots precess launches webots-bin internally who is the process that launches the controllers 
# that's why we need to get the pppid
proc = psutil.Process(os.getpid()) #current
parent = proc.parent() #parent
grandparent = parent.parent() if parent else None #grandparent
pppid = str(grandparent.pid)


simulation_rank = int(
    re.search(
        pppid + r" (\d+)",
        open("/tmp/autotech/simulationranks", "r").read(),
        re.MULTILINE
    ).group(1)
)


print(f"CLIENT ?{pppid}? {simulation_rank=}")



class WebotsVehicleManager:
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, vehicle_rank: int):
        self.vehicle_rank = vehicle_rank
        self.checkpoint_manager = CheckpointManager(supervisor, checkpoints[simulation_rank % n_map], vehicle_rank)

        self.v_min = 1 #np.random.rand()*2 + 0.5 # 0.5 to 2.5
        self.v_max = 9 #np.random.rand()*5 + 4   # 5 to 10
        basicTimeStep = int(supervisor.getBasicTimeStep())
        self.sensorTime = basicTimeStep // 4

        # negative value so that the first reset is not skipped
        self.last_reset = -1e6

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

        self.handler = logging.FileHandler(f"/tmp/autotech/Voiture_{self.simulation_rank}_{self.vehicle_rank}.log")
        self.handler.setFormatter(FORMATTER)
        self.log = logging.getLogger("SUPERVISOR")
        self.log.setLevel(level=LOG_LEVEL)
        self.log.addHandler(self.handler)


        self.log.debug("Connection to the vehicle")
        self.fifo_r = open(f"/tmp/autotech/{self.simulation_rank}_{self.vehicle_rank}tosupervisor.pipe", "rb")
        self.log.debug("Connection to the server")
        self.fifo_w = open(f"/tmp/autotech/{simulation_rank}_{vehicle_rank}toserver.pipe", "wb")
        

        self.translation_field = supervisor.getFromDef(f"TT02_{self.vehicle_rank}").getField("translation") # may cause access issues ...
        self.rotation_field = supervisor.getFromDef(f"TT02_{self.vehicle_rank}").getField("rotation") # may cause access issues ...

    # returns the lidar data of all vehicles
    def observe(self):
        # gets from Vehicle
        self.log.debug("trying to observe")
        obs = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize * (n_sensors + lidar_horizontal_resolution + camera_horizontal_resolution)), dtype=np.float32)
        self.log.info(f"observing {obs=}")
        return obs
    

    # reset the gym environment reset
    def reset(self, seed=None):
        self.log.debug("trying to reset vehicle")
        # this has to be done otherwise thec cars will shiver for a while sometimes when respawning and idk why
        if supervisor.getTime() - self.last_reset >= 1e-1:
            self.log.debug("getting info from vehicle")
            self.last_reset = supervisor.getTime()

            vehicle = supervisor.getFromDef(f"TT02_{self.vehicle_rank}")
            self.log.debug("resetting vehicle physics")
            self.checkpoint_manager.reset(seed)
            trans = self.checkpoint_manager.getTranslation()
            rot = self.checkpoint_manager.getRotation()

            self.translation_field.setSFVec3f(trans)
            self.rotation_field.setSFRotation(rot)
            self.checkpoint_manager.update()

            vehicle.resetPhysics()
            self.log.info("vehicle reset done")
            

        obs = np.zeros(n_sensors + lidar_horizontal_resolution + camera_horizontal_resolution, dtype=np.float32)
        info = {}
        return obs, info

    # step function of the gym environment
    def step(self):
        #action_steering = np.linspace(-.4, .4, n_actions_steering, dtype=np.float32)[action[0], None]
        #action_speed = np.linspace(self.v_min, self.v_max, n_actions_speed, dtype=np.float32)[action[1], None]

        # we should add a beacon sensor pointing upwards to detect the beacon
        self.log.debug("getting observation")
        obs = self.observe()
        self.log.info(f"observed {obs=}")
        sensor_data = obs[:n_sensors]
        reward = 0
        done = np.False_
        truncated = np.False_

        x, y, z = self.translation_field.getSFVec3f()
        b_past_checkpoint = self.checkpoint_manager.update(x, y)
        b_collided, = sensor_data # unpack sensor data

        if b_collided or (z < -10):
            #print(f"CLIENT{simulation_rank}/{self.vehicle_rank} : {b_collided=}, {z=}")
            reward = np.float32(0.0)
            done = np.True_
        elif b_past_checkpoint:
            reward = np.float32(1.0) #* np.cos(self.checkpoint_manager.getAngle() - self.rotation_field.getSFRotation()[3], dtype=np.float32)
            done = np.False_
        else:
            reward = np.float32(0.05)
            done = np.False_

        return obs, reward, done, truncated, {}


def main():

    envs = [WebotsVehicleManager(i) for i in range(n_vehicles)]
    print("CLIENT ALL : envs created")
    # check_env(env)

    logdir = "./Webots_tb/"

    supervisor.step()
    
    for i, e in enumerate(envs):
        e.log.debug("CLIENT : reset")
        e.reset(i)

    for i in range(n_vehicles, n_vehicles + n_stupid_vehicles):
        (
            supervisor
            .getFromDef(f"TT02_{i}")
            .getField("translation")
            .setSFVec3f(checkpoint_positions[i])
        )

    last_moved = [0 for _ in range(n_stupid_vehicles)]

    while supervisor.step() != -1:
        #print("CLIENT ALL : begin step")
        #Prédiction pour séléctionner une action à partir de l"observation
        for e in envs:
            obs, reward, done, truncated, info = e.step()
            if  done:
               obs, info = e.reset()

            e.log.info(f"sending {obs=}")
            e.fifo_w.write(obs.tobytes())
            e.log.info(f"sending {reward=}")
            e.fifo_w.write(reward.tobytes())
            e.log.info(f"sending {done=}")
            e.fifo_w.write(done.tobytes())
            e.log.info(f"sending {truncated=}")
            e.fifo_w.write(truncated.tobytes())
            e.fifo_w.flush()     


        for i in range(n_stupid_vehicles):
            tr_field = supervisor.getFromDef(f"TT02_{n_vehicles + i}").getField("translation")
            speed = np.linalg.norm(tr_field.getSFVec3f())

            if speed >= 0.1:
                last_moved[i] = supervisor.getTime()
            else:
                print("DIDNT MOVE")

            if supervisor.getTime() - last_moved[i]:
                print("DIDNT MOVE FOR 1s")
                tr_field.setSFVec3f(checkpoint_positions[0])




if __name__ == "__main__":
    print("BEGINS MAIN FROM CONTROLLER WORLD SUPERVISOR", flush=True)
    main()
