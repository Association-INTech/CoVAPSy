# just a file that lets us define some constants that are used in multiple files the simulation
import logging
from pathlib import Path
from typing import Any, Dict

import torch.nn as nn
from torch.cuda import is_available

from extractors import (  # noqa: F401
    CNN1DExtractor,
    CNN1DResNetExtractor,
    CNN1DResNetNoCamExtractor,
    TemporalResNetExtractor,
    DAVE2Extractor,
)

# Webots environments config
n_map = 4
n_simulations = 10
n_vehicles = 5
n_stupid_vehicles = 0
n_actions_steering = 16
n_actions_speed = 16
lidar_max_range = 12.0
respawn_on_crash = True  # whether to go backwards or to respawn when crashing


# Rewards
speed_window = 5     # average the speed over the last 5 checkpoints
speed_bonus = 0.05    # extra reward at reference speed (the checkpoint reward is 1.0)
collision_reward = -0.5  # penalty for colliding with another vehicle or obstacle
step_reward = 0.0  # reward for taking a step 
checkpoint_reward = 1.0  # reward for passing a checkpoint


# Camera config
camera_refresh_rate_divider = 4  # the camera is refreshed every sensorTime / camera


# Training config
device = "cuda" if is_available() else "cpu"
save_dir = Path("~/.cache/autotech").expanduser()
total_timesteps = 500_000
ppo_args: Dict[str, Any] = dict(
    n_steps=1024,
    n_epochs=10,
    batch_size=256,
    learning_rate=1e-3,
    gamma=0.99,
    verbose=1,
    normalize_advantage=True,
    device=device,
)



# Common extractor shared between the policy and value networks
# (cf: https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html)
ExtractorClass = DAVE2Extractor
context_size = ExtractorClass.context_size
lidar_horizontal_resolution = ExtractorClass.lidar_horizontal_resolution
camera_horizontal_resolution = ExtractorClass.camera_horizontal_resolution
camera__vertical_resolution = ExtractorClass.camera_vertical_resolution
n_sensors = ExtractorClass.n_sensors

if (
    lidar_horizontal_resolution != camera_horizontal_resolution
    and camera_horizontal_resolution != 0
):
    raise NotImplementedError(
        "Unhomogenous lidar and camera shape is only supported if camera_horizontal_resolution == 0"
    )

# Architecture of the model
policy_kwargs: Dict[str, Any] = dict(
    features_extractor_class=ExtractorClass,
    activation_fn=nn.ReLU,
    # Architecture of the MLP heads for the Value and Policy networks
    net_arch=[512, 512, 512],
)


# Logging config
LOG_LEVEL = logging.INFO
FORMATTER = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
