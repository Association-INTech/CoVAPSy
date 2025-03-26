import re
from typing import *
import numpy as np
import random
import gymnasium as gym
import time
import math

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv

from checkpoint import Checkpoint
from checkpointmanager import CheckpointManager, checkpoints

from controller import Supervisor
supervisor = Supervisor()

import torch.nn as nn

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

import matplotlib.pyplot as plt

def log(s: str):
    if B_DEBUG:
        print(s, file=open("/tmp/autotech/logs", "a"))


with open(f"/proc/{os.getppid()}/status") as f:
    log("youpi j'ai réussi à choper un truc I guess")
    for line in f:
        log(line)
        if line.startswith("PPid:"):
            log("found it bro")
            pppid = line.split()[1]
            log(pppid)
            break

log(pppid)

simulation_rank = int(
    re.search(
        pppid + r" (\d+)",
        open("/tmp/autotech/simulationranks", "r").read(),
        re.MULTILINE
    ).group(1)
)


log(f"CLIENT ?{pppid}? {simulation_rank=}")



class WebotsVehicleGymEnvironment(gym.Env):
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, vehicle_rank: int):
        self.vehicle_rank = vehicle_rank
        self.checkpoint_manager = CheckpointManager(supervisor, checkpoints, vehicle_rank)

        self.v_min = np.random.rand()*2 + 0.5 # 0.5 to 2.5
        self.v_max = 9 #np.random.rand()*5 + 4   # 5 to 10
        basicTimeStep = int(supervisor.getBasicTimeStep())
        self.sensorTime = basicTimeStep // 4

        # negative value so that the first reset is not skipped
        self.last_reset = -1e6

        # Emitter
        self.emitter = supervisor.getDevice(f"supervisor_emitter_{vehicle_rank}")
        self.emitter.setChannel(2 * self.vehicle_rank)

        # Receiver
        self.receiver = supervisor.getDevice(f"supervisor_receiver_{vehicle_rank}")
        self.receiver.enable(self.sensorTime)
        self.receiver.setChannel(2 * self.vehicle_rank + 1)


        log(f"CLIENT{simulation_rank}/{vehicle_rank} : begins init")
        log(f"CLIENT{simulation_rank}/{vehicle_rank} : {simulation_rank}toserver.pipe")
        self.fifo_w = open(f"/tmp/autotech/{simulation_rank}toserver.pipe", "wb")
        log(f"CLIENT{simulation_rank}/{vehicle_rank} : serverto{simulation_rank}.pipe")
        self.fifo_r = open(f"/tmp/autotech/serverto{simulation_rank}.pipe", "rb")

        # Last data received from the car
        self.last_data = np.zeros(n_sensors + lidar_horizontal_resolution + camera_horizontal_resolution, dtype=np.float32)

        self.translation_field = supervisor.getFromDef(f"TT02_{self.vehicle_rank}").getField("translation") # may cause access issues ...
        self.rotation_field = supervisor.getFromDef(f"TT02_{self.vehicle_rank}").getField("rotation") # may cause access issues ...

    # returns the lidar data of all vehicles
    def observe(self):
        # gets from Receiver
        if self.receiver.getQueueLength() > 0:
            while self.receiver.getQueueLength() > 1:
                self.receiver.nextPacket()
            self.last_data = np.frombuffer(self.receiver.getBytes(), dtype=np.float32)

        return self.last_data

    # reset the gym environment reset
    def reset(self, seed=None):
        # this has to be done otherwise thec cars will shiver for a while sometimes when respawning and idk why
        if supervisor.getTime() - self.last_reset >= 1e-1:
            self.last_reset = supervisor.getTime()

            vehicle = supervisor.getFromDef(f"TT02_{self.vehicle_rank}")

            self.checkpoint_manager.reset(seed)
            trans = self.checkpoint_manager.getTranslation()
            rot = self.checkpoint_manager.getRotation()

            self.translation_field.setSFVec3f(trans)
            self.rotation_field.setSFRotation(rot)
            self.checkpoint_manager.update()

            vehicle.resetPhysics()

        obs = self.observe()
        info = {}
        log(f"CLIENT{simulation_rank}/{self.vehicle_rank} : reset over")
        return obs, info

    # step function of the gym environment
    def step(self, action):
        action_steering = np.linspace(-.44, .44, n_actions_steering, dtype=np.float32)[action[0], None]
        action_speed = np.linspace(self.v_min, self.v_max, n_actions_speed, dtype=np.float32)[action[1], None]
        self.emitter.send(np.array([action_steering, action_speed], dtype=np.float32).tobytes())

        # we should add a beacon sensor pointing upwards to detect the beacon
        obs = self.observe()
        sensor_data = obs[:n_sensors]

        reward = 0
        done = np.False_
        truncated = np.False_

        x, y, z = self.translation_field.getSFVec3f()
        b_past_checkpoint = self.checkpoint_manager.update(x, y)
        b_collided, = sensor_data # unpack sensor data

        if b_collided or (z < -10):
            #print(f"CLIENT{simulation_rank}/{self.vehicle_rank} : {b_collided=}, {z=}")
            reward = np.float32(-2.0)
            done = np.True_
        elif b_past_checkpoint:
            reward = np.float32(1.0) #* np.cos(self.checkpoint_manager.getAngle() - self.rotation_field.getSFRotation()[3], dtype=np.float32)
            done = np.False_
        else:
            reward = np.float32(0.05)
            done = np.False_

        return obs, reward, done, truncated, {}


def main():

    envs = [WebotsVehicleGymEnvironment(i) for i in range(n_vehicles)]
    log(f"CLIENT ALL : envs created")
    # check_env(env)

    logdir = "./Webots_tb/"

    log(f"CLIENT ALL : {envs=}")
    supervisor.step()
    log("-------------------------------------------------------------------")
    for i, e in enumerate(envs):
        log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : reset")
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
        log(f"CLIENT ALL : begin step")
        #Prédiction pour séléctionner une action à partir de l"observation
        for e in envs:
            log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : trying to read from fifo")
            action = np.frombuffer(e.fifo_r.read(np.dtype(np.int64).itemsize * 2), dtype=np.int64)
            log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : received {action=}")

            obs, reward, done, truncated, info = e.step(action)

            if done:
                obs, info = e.reset()

            log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : sending {obs=}")
            e.fifo_w.write(obs.tobytes())
            log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : sending {reward=}")
            e.fifo_w.write(reward.tobytes())
            log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : sending {done=}")
            e.fifo_w.write(done.tobytes())
            log(f"CLIENT{simulation_rank}/{e.vehicle_rank} : sending {truncated=}")
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
