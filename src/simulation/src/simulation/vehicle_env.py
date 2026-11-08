import logging
import os
import subprocess
from typing import Dict, Tuple

import gymnasium as gym
import numpy as np

from webots import webots_path

from . import config as c

ObsType = Dict[str, np.ndarray]  # dict of 2D arrays (context_size, length)
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
        self.speed_history = np.array([], dtype=np.float32)  # to keep track of speed for reward adjustment
        self._initial_observation_pending = True  # flag to indicate if the initial observation has been received
        fifo_dir = c.FIFO_DIR
        os.makedirs(fifo_dir, exist_ok=True)
        self.handler = logging.FileHandler(
            f"{fifo_dir}/vehicle_{self.simulation_rank}_{self.vehicle_rank}.log"
        )
        self.handler.setFormatter(c.FORMATTER)
        self.log = logging.getLogger(
            f"SERVER_{self.simulation_rank}_{self.vehicle_rank}"
        )
        self.log.setLevel(level=c.LOG_LEVEL)
        self.log.addHandler(self.handler)

        self.log.info("Initialisation started")

        self.observation_space = gym.spaces.Dict({
            key: gym.spaces.Box(-30.0, 30.0, shape=(c.context_size, c.OBS_SPECS[key]), dtype=np.float32)
            for key in c.active_obs
        })

        self.obs_context = {
            key: np.zeros((c.context_size, c.OBS_SPECS[key]), dtype=np.float32)
            for key in c.active_obs
        }

        self.action_space = gym.spaces.MultiDiscrete(
            [c.n_actions_steering, c.n_actions_speed]
        )

        #self.log.debug("Creation of the pipes")

        for vr in ([ range(c.n_vehicles) ] if vehicle_rank == 0 else [ [vehicle_rank] ])[0]:
            for pipe_path in (
                f"{fifo_dir}/{simulation_rank}_{vr}toserver.pipe",
                f"{fifo_dir}/serverto{simulation_rank}_{vr}.pipe",
                f"{fifo_dir}/{simulation_rank}_{vr}tosupervisor.pipe",
            ):
                try:
                    os.mkfifo(pipe_path)
                except FileExistsError:
                    pass

        #  --mode=fast --minimize --no-rendering --batch --stdout
        if vehicle_rank == 0:
            webots_env = os.environ.copy()
            webots_log = open(f"{fifo_dir}/webots_{simulation_rank}.log", "w")
            proc = subprocess.Popen(
                [
                    "webots",
                    f"{webots_path}/worlds/piste{simulation_rank % c.n_map}.wbt",
                    "--mode=fast",
                    "--minimize",
                    "--batch",
                    "--no-rendering",
                    "--stdout",
                    "--stderr",
                ],
                env=webots_env,
                stdout=webots_log,
                stderr=webots_log,
            )
            with open(f"{fifo_dir}/simulationranks", "a") as f:
                f.write(f"{proc.pid} {simulation_rank}_{vehicle_rank}\n")

        #self.log.debug("Connection to the vehicle")
        self.fifo_w = open(
            f"{fifo_dir}/serverto{simulation_rank}_{vehicle_rank}.pipe", "wb"
        )
        #self.log.debug("Connection to the supervisor")
        self.fifo_r = open(
            f"{fifo_dir}/{simulation_rank}_{vehicle_rank}toserver.pipe", "rb"
        )

        #self.log.info("Initialisation finished\n")

    def _append_observation(self, cur_state: np.ndarray) -> dict[str, np.ndarray]:
        idx = c.n_sensors  # Skip the collision boolean at index 0
        for key in c.active_obs:
            length = c.OBS_SPECS[key]
            frame = cur_state[idx : idx + length][None, :]  # shape (1, length)
            if key == "Lidar":
                frame = np.nan_to_num(frame, nan=0.0, posinf=30.0, neginf=0.0)
                frame = np.clip(frame, 0.0, 30.0) # clip lidar data so inf / nan doesnt results in nan gradients in PyTorch
            self.obs_context[key] = np.concatenate([self.obs_context[key][1:], frame], axis=0)
            idx += length
        return {key: self.obs_context[key].copy() for key in c.active_obs}

    def reset(self, seed: int | None = None, **options) -> Tuple[ObsType, Dict]:
        
        # Repeat the last frame 5 times so Savitzky-Golay velocity starts cleanly at 0.0
        for key in c.active_obs:
            self.obs_context[key] = np.repeat(self.obs_context[key][-1:, :], c.context_size, axis=0)

        if self._initial_observation_pending:
            initial_state = np.frombuffer(
                self.fifo_r.read(
                    np.dtype(np.float32).itemsize
                    * (
                       c.obs_size
                    )
                ),
                dtype=np.float32,
            )
            obs = self._append_observation(initial_state)
            self._initial_observation_pending = False
            # Also repeat initial_state across all 5 slots on first boot so sg_velocity starts at 0.0
            for key in c.active_obs:
                self.obs_context[key] = np.repeat(self.obs_context[key][-1:, :], c.context_size, axis=0)
        
        info = {
            "sim_rank": self.simulation_rank,
            "vehicle_rank": self.vehicle_rank,
            "map": f"piste{self.simulation_rank % c.n_map}",
        }
        obs = {key: self.obs_context[key].copy() for key in c.active_obs}
        #self.log.info("reset finished\n")
        return obs, info

    def step(self, action: ActType):
        #self.log.info("Starting step")
        #self.log.info(f"sending {action=}")
        self.fifo_w.write(action.tobytes())
        self.fifo_w.flush()
        # communication with the supervisor
        #self.log.info(f"Waiting for (obs, reward, done, truncated) from SUPERVISOR_{self.simulation_rank}_{self.vehicle_rank}")
        cur_state = np.frombuffer(
            self.fifo_r.read(
                np.dtype(np.float32).itemsize
                * (
                    c.obs_size
                )
            ),
            dtype=np.float32,
        )
        #self.log.info(f"received {cur_state=}")
        reward = np.frombuffer(
            self.fifo_r.read(np.dtype(np.float32).itemsize), dtype=np.float32
        )[0]  # scalar
        #self.log.info(f"received {reward=}")
        done = np.frombuffer(
            self.fifo_r.read(np.dtype(np.bool_).itemsize), dtype=np.bool_
        )[0]  # scalar
        #self.log.info(f"received {done=}")
        truncated = np.frombuffer(
            self.fifo_r.read(np.dtype(np.bool_).itemsize), dtype=np.bool_
        )[0]  # scalar
        #self.log.info(f"received {truncated=}")

        obs = self._append_observation(cur_state)
        info = {
            "sim_rank": self.simulation_rank,
            "vehicle_rank": self.vehicle_rank,
            "map": f"piste{self.simulation_rank % c.n_map}",
        }
        #self.log.info("step over")

        return obs, reward, done, truncated, info
