import os
from logging import DEBUG
from typing import Any, Dict

import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv

from extractors import CNN1DResNetExtractor, TemporalResNetExtractor
from simulation import VehicleEnv
from simulation import config as c
from utils import onnx_utils

if __name__ == "__main__":
    if not os.path.exists("/tmp/autotech/"):
        os.mkdir("/tmp/autotech/")

    os.system('if [ -n "$(ls /tmp/autotech)" ]; then rm /tmp/autotech/*; fi')

    envs = SubprocVecEnv(
        [
            lambda sr=simulation_rank, vr=vehicle_rank: VehicleEnv(sr, vr)
            for vehicle_rank in range(c.n_vehicles)
            for simulation_rank in range(c.n_simulations)
        ]
    )

    ExtractorClass = CNN1DResNetExtractor

    policy_kwargs: Dict[str, Any] = dict(
        features_extractor_class=ExtractorClass,
        features_extractor_kwargs=dict(
            context_size=c.context_size,
            lidar_horizontal_resolution=c.lidar_horizontal_resolution,
            camera_horizontal_resolution=c.camera_horizontal_resolution,
            device=c.device,
        ),
        activation_fn=nn.ReLU,
        net_arch=[512, 512, 512],
    )

    ppo_args: Dict[str, Any] = dict(
        n_steps=4096,
        n_epochs=10,
        batch_size=256,
        learning_rate=3e-4,
        gamma=0.99,
        verbose=1,
        normalize_advantage=True,
        device=c.device,
    )

    save_path = (
        __file__.rsplit("/", 1)[0]
        + "~/.cache/autotech/checkpoints/"
        + ExtractorClass.__name__
        + "/"
    )
    os.makedirs(save_path, exist_ok=True)

    print(save_path)
    print(os.listdir(save_path))

    valid_files = [x for x in os.listdir(save_path) if x.rstrip(".zip").isnumeric()]

    if valid_files:
        model_name = max(valid_files, key=lambda x: int(x.rstrip(".zip")))
        print(f"Loading model {save_path + model_name}")
        model = PPO.load(
            save_path + model_name, envs, **ppo_args, policy_kwargs=policy_kwargs
        )
        i = int(model_name.rstrip(".zip")) + 1
        print(f"----- Model found, loading {model_name} -----")

    else:
        model = PPO("MlpPolicy", envs, **ppo_args, policy_kwargs=policy_kwargs)

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

    print("SERVER : finished executing")

    # obs = envs.reset()
    # while True:
    #     action, _states = model.predict(obs, deterministic=True)  # Use deterministic=True for evaluation
    #     obs, reward, done, info = envs.step(action)
    #     envs.render()  # Optional: visualize the environment

    while True:
        onnx_utils.export_onnx(
            model,
            f"~/.cache/autotech/model_{ExtractorClass.__name__}.onnx",
        )
        onnx_utils.test_onnx(model)

        if c.LOG_LEVEL <= DEBUG:
            # only used in debug mode
            from utils import PlotModelIO

            model.learn(
                total_timesteps=500_000,
                progress_bar=True,
                callback=PlotModelIO(),
            )
        else:
            model.learn(total_timesteps=500_000, progress_bar=True)

        print("iteration over")
        # TODO: we could just use a callback to save checkpoints or export the model to onnx
        model.save(save_path + str(i))

        i += 1
