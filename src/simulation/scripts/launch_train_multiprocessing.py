import os
from logging import DEBUG
from pathlib import Path
from typing import Any, Dict

import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback 

from extractors import (  # noqa: F401
    CNN1DExtractor,
    CNN1DResNetExtractor,
    TemporalResNetExtractor,
)
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

    save_path = c.save_dir / "checkpoints" / c.ExtractorClass.__name__

    save_path.mkdir(parents=True, exist_ok=True)

    valid_files = [x for x in save_path.iterdir() if x.name.rstrip(".zip").isnumeric()]

    if valid_files:
        model_path = max(valid_files, key=lambda x: int(x.name.rstrip(".zip")))
        print(f"Loading model {model_path.name}")
        model = PPO.load(model_path, envs, **c.ppo_args, policy_kwargs=c.policy_kwargs)
        i = int(model_path.name.rstrip(".zip")) + 1
        print(f"Model found, loading {model_path}")

    else:
        model = PPO("MlpPolicy", envs, **c.ppo_args, policy_kwargs=c.policy_kwargs)

        i = 0
        print("Model not found, creating a new one")

    print("model hyper parameters:")
    print(f"{model.learning_rate=}")
    print(f"{model.gamma=}")
    print(f"{model.verbose=}")
    print(f"{model.n_steps=}")
    print(f"{model.n_epochs=}")
    print(f"{model.batch_size=}")
    print(f"{model.device=}")

    # Save a checkpoint every 1000 steps
    checkpoint_callback = CheckpointCallback(
        save_freq=50_000,         # Fait des backups toutes les 100_000 itéartions
        save_path="Backups/",
        name_prefix="back_up_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
    )

    while True:
        onnx_utils.export_onnx(
            model,
            str(c.save_dir / f"model_{c.ExtractorClass.__name__}.onnx"),
        )
        onnx_utils.test_onnx(model)

        if c.LOG_LEVEL <= DEBUG:
            from utils import PlotModelIO

            model.learn(
                total_timesteps=c.total_timesteps,
                progress_bar=False,
                callback=[PlotModelIO(),checkpoint_callback],
            )
        else:

            
            model.learn(total_timesteps=c.total_timesteps, progress_bar=True,callback=checkpoint_callback)

        print("iteration over")
        # TODO: we could just use a callback to save checkpoints or export the model to onnx
        model.save(save_path / str(i))

        i += 1
