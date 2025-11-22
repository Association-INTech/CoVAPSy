import os
from typing import *
import numpy as np
import gymnasium as gym

from config import *


def log(s: str):
    if B_DEBUG:
        print(s, file=open("/tmp/autotech/logs", "a"))
class WebotsSimulationGymEnvironment(gym.Env):
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, simulation_rank: int, vehicle_rank: int):
        super().__init__()
        self.simulation_rank = simulation_rank
        self.vehicle_rank = vehicle_rank

        # this is only true if lidar_horizontal_resolution = camera_horizontal_resolution
        box_min = np.zeros([2, context_size, lidar_horizontal_resolution], dtype=np.float32)
        box_max = np.ones([2, context_size, lidar_horizontal_resolution], dtype=np.float32) * 30

        self.observation_space = gym.spaces.Box(box_min, box_max, dtype=np.float32)
        self.action_space = gym.spaces.MultiDiscrete([n_actions_steering, n_actions_speed])

        if not os.path.exists("/tmp/autotech"):
            os.mkdir("/tmp/autotech")

        log(f"SERVER{simulation_rank}_{vehicle_rank} : {simulation_rank}_{vehicle_rank}")

        os.mkfifo(f"/tmp/autotech/{simulation_rank}_{vehicle_rank}toserver.pipe")
        os.mkfifo(f"/tmp/autotech/serverto{simulation_rank}_{vehicle_rank}.pipe")

        #  --mode=fast --minimize --no-rendering --batch --stdout
        os.system(f"""
            webots {__file__.rsplit('/', 1)[0]}/worlds/piste{simulation_rank % n_map}.wbt --mode=fast --minimize --no-rendering --batch --stdout &
            echo $! {simulation_rank}_{vehicle_rank} >>/tmp/autotech/simulationranks
        """)
        log(f"SERVER{simulation_rank}_{vehicle_rank} : {simulation_rank}_{vehicle_rank}toserver.pipe")
        self.fifo_r = open(f"/tmp/autotech/{simulation_rank}_{vehicle_rank}toserver.pipe", "rb")
        log(f"SERVER{simulation_rank}_{vehicle_rank} : serverto{simulation_rank}_{vehicle_rank}.pipe")
        self.fifo_w = open(f"/tmp/autotech/serverto{simulation_rank}_{vehicle_rank}.pipe", "wb")
        log(f"SERVER{simulation_rank}_{vehicle_rank} : fifo opened :D and init done")
        log("-------------------------------------------------------------------")

    def reset(self, seed=0):
        # basically useless function

        # lidar data
        # this is true for lidar_horizontal_resolution = camera_horizontal_resolution
        self.context = obs = np.zeros([2, context_size, lidar_horizontal_resolution], dtype=np.float32)
        info = {}
        return obs, info

    def step(self, action):
        log(f"SERVER{self.simulation_rank}_{self.vehicle_rank} : sending {action=}")
        self.fifo_w.write(action.tobytes())
        self.fifo_w.flush()

        # communication with the supervisor
        cur_state   = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize * (n_sensors + lidar_horizontal_resolution + camera_horizontal_resolution)), dtype=np.float32)
        log(f"SERVER{self.simulation_rank}_{self.vehicle_rank} : received {cur_state=}")
        reward      = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize), dtype=np.float32)[0] # scalar
        log(f"SERVER{self.simulation_rank}_{self.vehicle_rank} : received {reward=}")
        done        = np.frombuffer(self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool)[0] # scalar
        log(f"SERVER{self.simulation_rank}_{self.vehicle_rank} : received {done=}")
        truncated   = np.frombuffer(self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool)[0] # scalar
        log(f"SERVER{self.simulation_rank}_{self.vehicle_rank} : received {truncated=}")
        info        = {}

        cur_state = np.nan_to_num(cur_state[n_sensors:], nan=0., posinf=30.)

        lidar_obs = cur_state[:lidar_horizontal_resolution]
        camera_obs = cur_state[lidar_horizontal_resolution:]

        # apply dropout to the camera
        # p = 0.5
        # camera_obs *= np.random.binomial(1, 1-p, camera_obs.shape) # random values in {0, 1}

        self.context = obs = np.concatenate([
            self.context[:, 1:],
            [lidar_obs[None], camera_obs[None]]
        ], axis=1)
        # check if the context is correct
        # if self.simulation_rank == 0:
        #     print(f"{(obs[0] == 0).mean():.3f} {(obs[1] == 0).mean():.3f}")
        return obs, reward, done, truncated, info


