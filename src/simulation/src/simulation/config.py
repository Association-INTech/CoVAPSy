# just a file that lets us define some constants that are used in multiple files the simulation
import logging
from pathlib import Path
from typing import Any, Dict
import os

import torch.nn as nn
from torch.cuda import is_available

from extractors import (  # noqa: F401

    CNN1DAlexNetExtractor,
    CNN1DResNetExtractor,
    CNN1DResNetExtractorMemory,
    CNN1DResNetNoCamExtractor,
    TemporalResNetExtractor,
)



# Webots environments config
# Set to an int (0 = piste0, 1 = piste1 / map 2, 2 = piste2, 3 = piste3) to lock a single map,
# or set to None to cycle across n_map using (simulation_rank % n_map)
map_id: int | None = 1  # <-- 1 selects Map 2 (0-indexed: 0 is Map 1, 1 is Map 2)
map_id = 1
n_simulations = 1
n_vehicles = 2
n_stupid_vehicles = 1
n_actions_steering = 16
n_actions_speed = 16
lidar_max_range = 12.0
v_min = 1.0
v_max = 9.0
target_speed = 5.0  # m/s
speed_factor = 2.0  # reward for maintaining target speed
overtaking_factor = 0.5
steer_reward = 0.02  # penalty scale on |delta_steer| (rad/step); subtracted from step reward
respawn_on_crash = True  # whether to go backwards or to respawn when crashing
continuous_actions:bool = True

# Fifo model specifications
ARCH_LIST = ["res", "resmemo", "alex"]
MODEL_ARCH = os.environ.get("AUTOTECH_ARCH", "resmemo")
# Automatically gives 0 for resnet, 100 for alexnet, 200 for cnn1d, etc.!
PORT_OFFSET = (ARCH_LIST.index(MODEL_ARCH) if MODEL_ARCH in ARCH_LIST else 0) * 100
FIFO_DIR = f"/tmp/autotech/{MODEL_ARCH}"

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
save_dir = Path("~/.cache/autotech/").expanduser()
total_timesteps = 100_000

sac_args = dict(
    learning_rate=3e-4,
    buffer_size=6_000,       # Stores 10k steps (~70 min of Webots driving)
    batch_size=128,            # GPU samples transitions per update
    learning_starts=1_000,     # Collect 1000 random steps before first GPU update
    tau=0.005,                 # Soft target network update rate
    gamma=0.99,                # Horizon reward coeff
    train_freq=(16, "step"),    # Update on GPU every 4 Webots steps
    gradient_steps=4,
    ent_coef="auto",           # Magic: SAC tunes entropy temperature alpha automatically!
    verbose=1,
    device=device,
)

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
EXTRACTOR_CLASSES = {
    "res": CNN1DResNetExtractor,
    "alex": CNN1DAlexNetExtractor,
    "resmemo": CNN1DResNetExtractorMemory,
}
try:
    ExtractorClass = EXTRACTOR_CLASSES[MODEL_ARCH]
except KeyError as exc:
    raise ValueError(
        f"Unsupported AUTOTECH_ARCH {MODEL_ARCH!r}; expected one of {ARCH_LIST}"
    ) from exc
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
    #"Internal_Speed": 1,
    #"Internal_Steer": 1,
}
active_obs = [
    "Collision",
    "Lidar",
    "Cam",
    "Speed_Cmd",
    "Steer_Cmd",
    #"Internal_Speed",
    #"Internal_Steer",
    ]
obs_size = sum(OBS_SPECS[k] for k in active_obs)



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
    share_features_extractor=True,
)


# Logging config
LOG_LEVEL = logging.INFO
FORMATTER = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
