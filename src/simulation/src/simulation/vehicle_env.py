import logging
import os
import subprocess
from typing import Dict, Tuple

import gymnasium as gym
import numpy as np

from webots import webots_path

from . import config as c

ObsType = np.ndarray  # float32 array
ActType = np.ndarray  # int64 array


class VehicleEnv(gym.Env):
    """
    Gymnasium environment representing a single vehicle controlled inside
    a Webots simulation.

    Creating an instance of this environment sets up all required IPC
    mechanisms (named pipes) and may spawn a new Webots process, depending
    on the vehicle index.

    Communication is organized as a closed control loop:

    VehicleEnv
        |
        |   action
        ▼
    controller_vehicle_driver
        |
        |   observation
        ▼
    controller_world_supervisor (shared by vehicle of the same simulation)
        |
        |   observation, reward, done, truncated
        ▼
    VehicleEnv
    """

    def __init__(self, simulation_rank: int, vehicle_rank: int):
        super().__init__()
        self.simulation_rank = simulation_rank
        self.vehicle_rank = vehicle_rank

        self.handler = logging.FileHandler(
            f"/tmp/autotech/Voiture_{self.simulation_rank}_{self.vehicle_rank}.log"
        )
        self.handler.setFormatter(c.FORMATTER)
        self.log = logging.getLogger(
            f"SERVER_{self.simulation_rank}_{self.vehicle_rank}"
        )
        self.log.setLevel(level=c.LOG_LEVEL)
        self.log.addHandler(self.handler)

        self.log.info("Initialisation started")

        # this is only true if lidar_horizontal_resolution = camera_horizontal_resolution
        box_min = np.zeros(
            [2, c.context_size, c.lidar_horizontal_resolution], dtype=np.float32
        )
        box_max = (
            np.ones(
                [2, c.context_size, c.lidar_horizontal_resolution], dtype=np.float32
            )
            * 30
        )

        self.observation_space = gym.spaces.Box(box_min, box_max, dtype=np.float32)
        self.action_space = gym.spaces.MultiDiscrete(
            [c.n_actions_steering, c.n_actions_speed]
        )

        if not os.path.exists("/tmp/autotech"):
            os.mkdir("/tmp/autotech")

        self.log.debug("Creation of the pipes")

        os.mkfifo(f"/tmp/autotech/{simulation_rank}_{vehicle_rank}toserver.pipe")
        os.mkfifo(f"/tmp/autotech/serverto{simulation_rank}_{vehicle_rank}.pipe")
        os.mkfifo(f"/tmp/autotech/{simulation_rank}_{vehicle_rank}tosupervisor.pipe")

        #  --mode=fast --minimize --no-rendering --batch --stdout
        if vehicle_rank == 0:
            proc = subprocess.Popen(
                [
                    "webots",
                    f"{webots_path}/worlds/piste{simulation_rank % c.n_map}.wbt",
                    "--mode=fast",
                    "--minimize",
                    "--batch",
                    "--stdout",
                ]
            )

            with open("/tmp/autotech/simulationranks", "a") as f:
                f.write(f"{proc.pid} {simulation_rank}_{vehicle_rank}\n")

        self.log.debug("Connection to the vehicle")
        self.fifo_w = open(
            f"/tmp/autotech/serverto{simulation_rank}_{vehicle_rank}.pipe", "wb"
        )
        self.log.debug("Connection to the supervisor")
        self.fifo_r = open(
            f"/tmp/autotech/{simulation_rank}_{vehicle_rank}toserver.pipe", "rb"
        )

        self.log.info("Initialisation finished\n")

    def reset(self, seed: int | None = None, **options) -> Tuple[ObsType, Dict]:
        # basically useless function

        # lidar data
        # this is true for lidar_horizontal_resolution = camera_horizontal_resolution
        self.context = obs = np.zeros(
            [2, c.context_size, c.lidar_horizontal_resolution], dtype=np.float32
        )
        info = {}
        self.log.info("reset finished\n")
        return obs, info

    def step(self, action: ActType):
        self.log.info("Starting step")
        self.log.info(f"sending {action=}")
        self.fifo_w.write(action.tobytes())
        self.fifo_w.flush()

        # communication with the supervisor
        self.log.debug("trying to get info from supervisor")
        cur_state = np.frombuffer(
            self.fifo_r.read(
                np.dtype(np.float32).itemsize
                * (
                    c.n_sensors
                    + c.lidar_horizontal_resolution
                    + c.camera_horizontal_resolution
                )
            ),
            dtype=np.float32,
        )
        self.log.info(f"received {cur_state=}")
        reward = np.frombuffer(
            self.fifo_r.read(np.dtype(np.float32).itemsize), dtype=np.float32
        )[0]  # scalar
        self.log.info(f"received {reward=}")
        done = np.frombuffer(
            self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool
        )[0]  # scalar
        self.log.info(f"received {done=}")
        truncated = np.frombuffer(
            self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool
        )[0]  # scalar
        self.log.info(f"received {truncated=}")
        info = {}

        cur_state = np.nan_to_num(cur_state[c.n_sensors :], nan=0.0, posinf=30.0)

        lidar_obs = cur_state[: c.lidar_horizontal_resolution]
        camera_obs = cur_state[c.lidar_horizontal_resolution :]

        self.context = obs = np.concatenate(
            [self.context[:, 1:], [lidar_obs[None], camera_obs[None]]], axis=1
        )

        self.log.info("step over")

        return obs, reward, done, truncated, info
