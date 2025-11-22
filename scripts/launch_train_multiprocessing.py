import os
import sys

from typing import *

import torch.nn as nn

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

simu_path = __file__.rsplit('/', 2)[0] + '/src/Simulateur'
print(f"{simu_path = }")
if simu_path not in sys.path:
    sys.path.insert(0, simu_path)

from config import *
from TemporalResNetExtractor import TemporalResNetExtractor
from onnx_utils import *

from WebotsSimulationGymEnvironment import WebotsSimulationGymEnvironment
if B_DEBUG: from DynamicActionPlotCallback import DynamicActionPlotDistributionCallback

def log(s: str):
    if B_DEBUG:
        print(s, file=open("/tmp/autotech/logs", "a"))



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
        net_arch=[512, 512, 512],
    )


    ppo_args = dict(
        n_steps=4096,
        n_epochs=10,
        batch_size=256,
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
            model.learn(total_timesteps=500_000, callback=DynamicActionPlotDistributionCallback())
        else:
            model.learn(total_timesteps=500_000)

        model.save(save_path + str(i))

        i += 1
