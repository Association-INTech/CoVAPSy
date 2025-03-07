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

from config import *
from extractors.CNN1DExtractor import CNN1DExtractor
from extractors.TemporalResNetExtractor import TemporalResNetExtractor

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
        lidar_min = np.zeros([context_size, lidar_horizontal_resolution], dtype=np.float32)
        lidar_max = np.ones([context_size, lidar_horizontal_resolution], dtype=np.float32) * 30

        camera_min = -np.ones([context_size, camera_horizontal_resolution], dtype=np.float32)
        camera_max = np.ones([context_size, camera_horizontal_resolution], dtype=np.float32)

        box_min = np.concatenate([lidar_min, camera_min], axis=1)
        box_max = np.concatenate([lidar_max, camera_max], axis=1)

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
        self.context = obs = np.zeros([context_size, (lidar_horizontal_resolution + camera_horizontal_resolution)], dtype=np.float32)
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

        self.context = obs = np.concatenate([self.context[1:], np.nan_to_num(cur_state[n_sensors:], nan=0., posinf=30.)[None]])

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

    ExtractorClass = TemporalResNetExtractor

    policy_kwargs = dict(
        features_extractor_class=ExtractorClass,
        features_extractor_kwargs=dict(
            context_size=context_size,
            lidar_horizontal_resolution=lidar_horizontal_resolution,
            camera_horizontal_resolution=camera_horizontal_resolution,
            device=device
        ),
        activation_fn=nn.ReLU,
        net_arch=[1024, 1024],
    )


    ppo_args = dict(
        n_steps=2048,
        n_epochs=10,
        batch_size=512,
        learning_rate=3e-4,
        gamma=0.99,
        verbose=1,
        normalize_advantage=True,
        device=device
    )


    save_path = __file__.rsplit("/", 1)[0] + "/checkpoints/" + ExtractorClass.__name__ + "/"
    if not os.path.exists(save_path):
        os.mkdir(save_path)


    # will throw an error if the directory is emptye
    # else will be the name of the last checkpoint
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
        # os.system(
        #     f'''if [ -n "$(ls {save_path})" ]; then
        #         rm {save_path}*
        #     fi'''
        # )

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
    print(f"{model.policy=}")


    # NOTE: this is required for the ``fork`` method to work
    # same object as pi_features_extractor and vf_features_extractor
    #model.policy.share_memory()

    log(f"SERVER : finished executing")
    # keep the process running and the fifo open

    while True:
        true_model = nn.Sequential(
            model.policy.features_extractor.net,
            model.policy.mlp_extractor.policy_net,
            model.policy.action_net
        ).to("cpu")
        true_model.eval()
        x = torch.randn(1, 2, 128, 128)

        # save as onnx
        torch.onnx.export(
            true_model,
            x,
            "model.onnx",
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}}
        )
        true_model.to(device)
        true_model.train()

        if B_DEBUG:
            model.learn(total_timesteps=100_000, callback=DynamicActionPlotDistributionCallback())
        else:
            model.learn(total_timesteps=100_000)

        model.save(save_path + str(i))

        i += 1
