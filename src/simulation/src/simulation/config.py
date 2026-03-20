# just a file that lets us define some constants that are used in multiple files the simulation
import logging
from typing import Any, Dict

from torch.cuda import is_available

from extractors import (  # noqa: F401
    CNN1DExtractor,
    CNN1DResNetExtractor,
    TemporalResNetExtractor,
)

# Webots environments config
n_map = 2
n_simulations = 1
n_vehicles = 2
n_stupid_vehicles = 0
n_actions_steering = 16
n_actions_speed = 16
lidar_max_range = 12.0
device = "cuda" if is_available() else "cpu"


# Training config
total_timesteps = 500_000
ppo_args: Dict[str, Any] = dict(
    n_steps=4096,
    n_epochs=10,
    batch_size=256,
    learning_rate=3e-4,
    gamma=0.99,
    verbose=1,
    normalize_advantage=True,
    device=device,
)
ExtractorClass = TemporalResNetExtractor
context_size = ExtractorClass.context_size
lidar_horizontal_resolution = ExtractorClass.lidar_horizontal_resolution
camera_horizontal_resolution = ExtractorClass.camera_horizontal_resolution
n_sensors = ExtractorClass.n_sensors


# Logging config
LOG_LEVEL = logging.INFO
FORMATTER = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
