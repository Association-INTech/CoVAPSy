import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class CameraOnlyExtractor(BaseFeaturesExtractor):
    # Read by config.py on the class itself (before any instance exists) to
    # build the environment's observation space, same as in the template.
    lidar_horizontal_resolution = 1080
    camera_horizontal_resolution = 128
    camera_vertical_resolution = 64
    n_sensors = 1

    features_dim_out = 64

    input_type = "image"

    def __init__(
        self,
        space: spaces.Box,
        device: str = "cpu",
    ):
        in_channels = space.shape[0]
        cnn = nn.Sequential(
            nn.Conv2d(in_channels, 16, kernel_size=5, stride=2, padding=2, device=device),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1, device=device),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1, device=device),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(64, self.features_dim_out, device=device),
            nn.ReLU(),
        )

        super().__init__(space, self.features_dim_out)
        self.cnn = cnn

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.cnn(observations)