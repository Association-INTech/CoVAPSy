import os
import time
from typing import *

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.multiprocessing as mp

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv

import gymnasium as gym

from onnx_utils import export_onnx, test_onnx
from config import *
from CNN1DExtractor import CNN1DExtractor
from TemporalResNetExtractor import TemporalResNetExtractor
from CNN1DResNetExtractor import CNN1DResNetExtractor

if B_DEBUG: from DynamicActionPlotCallback import DynamicActionPlotDistributionCallback


def log(s: str):
    if B_DEBUG:
        print(s, file=open("/tmp/autotech/logs", "a"))

class WebotsSimulationGymEnvironment(gym.Env):
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, simulation_rank: int):
        super().__init__()
        self.simulation_rank = simulation_rank

        # this is only true if lidar_horizontal_resolution = camera_horizontal_resolution
        box_min = np.zeros([2, context_size, lidar_horizontal_resolution], dtype=np.float32)
        box_max = np.ones([2, context_size, lidar_horizontal_resolution], dtype=np.float32) * 30

        self.observation_space = gym.spaces.Box(box_min, box_max, dtype=np.float32)
        self.action_space = gym.spaces.MultiDiscrete([n_actions_steering, n_actions_speed])

        if not os.path.exists("/tmp/autotech"):
            os.mkdir("/tmp/autotech")

        log(f"SERVER{simulation_rank} : {simulation_rank=}")

        os.mkfifo(f"/tmp/autotech/{simulation_rank}toserver.pipe")
        os.mkfifo(f"/tmp/autotech/serverto{simulation_rank}.pipe")

        #  --mode=fast --minimize --no-rendering --batch --stdout
        os.system(f"""
            webots {__file__.rsplit('/', 1)[0]}/worlds/piste2.wbt --mode=fast --minimize --no-rendering --batch --stdout &
            echo $! {simulation_rank} >>/tmp/autotech/simulationranks
        """)
        log(f"SERVER{simulation_rank} : {simulation_rank}toserver.pipe")
        self.fifo_r = open(f"/tmp/autotech/{simulation_rank}toserver.pipe", "rb")
        log(f"SERVER{simulation_rank} : serverto{simulation_rank}.pipe")
        self.fifo_w = open(f"/tmp/autotech/serverto{simulation_rank}.pipe", "wb")
        log(f"SERVER{simulation_rank} : fifo opened :D and init done")
        log("-------------------------------------------------------------------")

    def reset(self, seed=0):
        # basically useless function

        # lidar data
        # this is true for lidar_horizontal_resolution = camera_horizontal_resolution
        self.context = obs = np.zeros([2, context_size, lidar_horizontal_resolution], dtype=np.float32)
        info = {}
        return obs, info

    def step(self, action):
        log(f"SERVER{self.simulation_rank} : sending {action=}")
        self.fifo_w.write(action.tobytes())
        self.fifo_w.flush()

        # communication with the supervisor
        cur_state   = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize * (n_sensors + lidar_horizontal_resolution + camera_horizontal_resolution)), dtype=np.float32)
        log(f"SERVER{self.simulation_rank} : received {cur_state=}")
        reward      = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize), dtype=np.float32)[0] # scalar
        log(f"SERVER{self.simulation_rank} : received {reward=}")
        done        = np.frombuffer(self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool)[0] # scalar
        log(f"SERVER{self.simulation_rank} : received {done=}")
        truncated   = np.frombuffer(self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool)[0] # scalar
        log(f"SERVER{self.simulation_rank} : received {truncated=}")
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


if __name__ == "__main__":
    if not os.path.exists("/tmp/autotech/"):
        os.mkdir("/tmp/autotech/")

    os.system('if [ -n "$(ls /tmp/autotech)" ]; then rm /tmp/autotech/*; fi')
    if B_DEBUG:
        print("Webots started", file=open("/tmp/autotech/logs", "w"))

    def make_env(rank: int):
        log(f"CAREFUL !!! created an SERVER env with {rank=}")
        return WebotsSimulationGymEnvironment(rank)

    envs = SubprocVecEnv([lambda rank=rank : make_env(rank) for rank in range(n_simulations)])

    ExtractorClass = CNN1DResNetExtractor

    policy_kwargs = dict(
        features_extractor_class=ExtractorClass,
        features_extractor_kwargs=dict(
            context_size=context_size,
            lidar_horizontal_resolution=lidar_horizontal_resolution,
            camera_horizontal_resolution=camera_horizontal_resolution,
            device=device
        ),
        activation_fn=nn.ReLU,
        net_arch=[512, 512, 512],
    )


    ppo_args = dict(
        n_steps=2048,
        n_epochs=10,
        batch_size=128,
        learning_rate=3e-4,
        gamma=0.99,
        verbose=1,
        normalize_advantage=True,
        device=device
    )


    save_path = __file__.rsplit("/", 1)[0] + "/checkpoints/" + ExtractorClass.__name__ + "/"
    if not os.path.exists(save_path):
        os.mkdir(save_path)

    print(save_path)
    print(os.listdir(save_path))

    valid_files = [x for x in os.listdir(save_path) if x.rstrip(".zip").isnumeric()]

    if valid_files:
        model_name = max(
            valid_files,
            key=lambda x : int(x.rstrip(".zip"))
        )
        print(f"Loading model {save_path + model_name}")
        model = PPO.load(
            save_path + model_name,
            envs,
            **ppo_args,
            policy_kwargs=policy_kwargs
        )
        i = int(model_name.rstrip(".zip")) + 1
        print(f"----- Model found, loading {model_name} -----")

    else:
        model = PPO(
            "MlpPolicy",
            envs,
            **ppo_args,
            policy_kwargs=policy_kwargs
        )

        i = 0
        print("----- Model not found, creating a new one -----")

    print("MODEL HAS HYPER PARAMETERS:")
    print(f"{model.learning_rate=}")
    print(f"{model.gamma=}")
    print(f"{model.verbose=}")
    print(f"{model.n_steps=}")
    print(f"{model.n_epochs=}")
    print(f"{model.batch_size=}")
    print(f"{model.device=}")

    log(f"SERVER : finished executing")

    # obs = envs.reset()
    # while True:
    #     action, _states = model.predict(obs, deterministic=True)  # Use deterministic=True for evaluation
    #     obs, reward, done, info = envs.step(action)
    #     envs.render()  # Optional: visualize the environment


    while True:
        export_onnx(model)
        test_onnx(model)

        if B_DEBUG:
            model.learn(total_timesteps=100_000, callback=DynamicActionPlotDistributionCallback())
        else:
            model.learn(total_timesteps=100_000)

        model.save(save_path + str(i))

        i += 1
