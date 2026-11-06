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


"""
    New architecture of the observation space:

    - Config defines the active_obs, which is a list of the observation fields that are actually used in the environment.
    - OBS_SPECS defines the length of each observation field in 1 frame so one observation has the shape : 
    Dict[str, np.ndarray] where 
            - each key is an observation field in active_obs
            - the value is a 1D array of length OBS_SPECS[key].
    - Env holds self.obs_context, which is a dictionary of the active_obs, each of which is a 3D array of shape (context_size, 1, length)
    - Each field of obs is considered as float32 inside a pipe
    - For example if active_obs = ["Lidar", "Cam", "Collision"], then 
    - the pipe sends (1080 + 1080 + 1) * 4 bytes
    
"""

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

# Master registry of every possible observation field and its 1-frame length
OBS_SPECS = {
    "Collision": n_sensors,
    "Lidar": lidar_horizontal_resolution,
    "Cam": camera_horizontal_resolution,
    # "Internal_Speed": 1,  # Future wheel-encoder speed from Webots
    "Speed_Cmd": 1,       # Self-known in VehicleEnv
    "Steer_Cmd": 1,       # Self-known in VehicleEnv
}
active_obs = ["Lidar", "Cam", "Speed_Cmd", "Steer_Cmd"]
# collision is still active but we don't need to carry the information that it was 1 as the pipe is reset anyway after a crash
obs_size = n_sensors + sum(OBS_SPECS[k] for k in active_obs)



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
