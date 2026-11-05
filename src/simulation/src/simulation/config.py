# just a file that lets us define some constants that are used in multiple files the simulation
import logging
from pathlib import Path
from typing import Any, Dict

import torch.nn as nn
from torch.cuda import is_available

from extractors import (  # noqa: F401

    CNN1DAlexNetExtractor,
    CNN1DResNetExtractor,
    CNN1DResNetNoCamExtractor,
    TemporalResNetExtractor,
)

# Webots environments config
n_map = 1
n_simulations = 1
n_vehicles = 2
n_stupid_vehicles = 0
n_actions_steering = 16
n_actions_speed = 16
lidar_max_range = 12.0
v_min = 1.0
v_max = 9.0
target_speed = 5.0  # m/s
speed_factor = 2.0  # reward for maintaining target speed
respawn_on_crash = True  # whether to go backwards or to respawn when crashing
#extra_context_channels = 2  # number of extra context channels (speed_command, steering_command)
active_obs = ["Lidar", "Cam", "Collision", "Speed_Cmd", "Steer_Cmd"]

# Master registry of every possible observation field and its 1-frame length
OBS_SPECS = {
    "Collision": 1,
    "Lidar": 1080,
    "Cam": 1080,
    # "Internal_Speed": 1,  # Future wheel-encoder speed from Webots
    "Speed_Cmd": 1,       # Self-known in VehicleEnv
    "Steer_Cmd": 1,       # Self-known in VehicleEnv
}

# Only fields that originate inside Webots cross the FIFO pipe:
WEBOTS_PIPE_FIELDS = ["Collision"] + [
    k for k in active_obs if k in ("Lidar", "Cam", "Internal_Speed")
]

# Training config
device = "cuda" if is_available() else "cpu"
save_dir = Path("~/.cache/autotech").expanduser()
total_timesteps = 100_000
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
ExtractorClass = CNN1DResNetExtractor
context_size = ExtractorClass.context_size
lidar_horizontal_resolution = ExtractorClass.lidar_horizontal_resolution
camera_horizontal_resolution = ExtractorClass.camera_horizontal_resolution
sensor_channel_count = 1 + (camera_horizontal_resolution != 0)
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
    net_arch=[64, 64],
)


# Logging config
LOG_LEVEL = logging.INFO
FORMATTER = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
