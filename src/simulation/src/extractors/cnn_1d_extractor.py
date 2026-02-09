import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class CNN1DExtractor(BaseFeaturesExtractor):
    context_size = 1
    lidar_horizontal_resolution = 1080
    camera_horizontal_resolution = 1080
    n_sensors = 1

    # just an alias to avoid confusion because
    # the lidar and camera have the same resolution
    horizontal_resolution = 1080

    def __init__(
        self,
        space: spaces.Box,
        device: str = "cpu",
    ):
        cnn = nn.Sequential(
            # shape = [batch_size, 2, 1080]
            nn.Conv1d(2, 64, kernel_size=7, stride=2, padding=3, device=device),
            nn.ReLU(),
            nn.MaxPool1d(3),
            nn.Dropout1d(0.2),
            # shape = [batch_size, 64, 180]
            nn.Conv1d(64, 64, kernel_size=3, padding="same", device=device),
            nn.ReLU(),
            nn.MaxPool1d(3),
            nn.Dropout1d(0.3),
            # shape = [batch_size, 64, 60]
            nn.Conv1d(64, 128, kernel_size=3, padding="same", device=device),
            nn.ReLU(),
            nn.AvgPool1d(2),
            nn.Dropout1d(0.4),
            # shape = [batch_size, 128, 30]
            nn.Conv1d(128, 128, kernel_size=3, padding="same", device=device),
            nn.ReLU(),
            nn.AvgPool1d(2),
            # shape = [batch_size, 128, 15]
            nn.Flatten(),
            nn.Dropout(0.5),
            # shape = [batch_size, 1920]
        )

        # Compute shape by doing one forward pass
        with torch.no_grad():
            n_flatten = cnn(
                torch.zeros([1, 2, self.horizontal_resolution], device=device)
            ).shape[1]

        super().__init__(space, n_flatten)

        # we cannot assign this directly to self.cnn before calling the super constructor
        self.net = cnn

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        # strip the context out
        observations = observations[..., 0, :]
        return self.net(observations)
