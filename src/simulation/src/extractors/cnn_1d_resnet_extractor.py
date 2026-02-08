import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class CNN1DResNetExtractor(BaseFeaturesExtractor):
    context_size = 1
    lidar_horizontal_resolution = 1024
    camera_horizontal_resolution = 1024
    n_sensors = 1

    # just an alias to avoid confusion because
    # the lidar and camera have the same resolution
    horizontal_resolution = 1024

    def __init__(
        self,
        space: spaces.Box,
        device: str = "cpu",
    ):
        net = nn.Sequential(
            # shape = [batch_size, 2, 1024]
            Compressor(device),
            # shape = [batch_size, 64, 256]
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 64, downsample=True, device=device),
            # shape = [batch_size, 128, 128]
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 128, downsample=True, device=device),
            # shape = [batch_size, 128, 64]
            ResidualBlock(128, 128, device=device),
            ResidualBlock(128, 128, device=device),
            ResidualBlock(128, 128, downsample=True, device=device),
            # shape = [batch_size, 256, 32]
            ResidualBlock(128, 128, device=device),
            ResidualBlock(128, 128, device=device),
            ResidualBlock(128, 256, downsample=True, device=device),
            # shape = [batch_size, 256, 16]
            ResidualBlock(256, 256, device=device),
            ResidualBlock(256, 256, device=device),
            ResidualBlock(256, 256, downsample=True, device=device),
            # shape = [batch_size, 256, 8]
            nn.AvgPool1d(8),
            # shape = [batch_size, 256, 1]
            nn.Flatten(),
            # shape = [batch_size, 256]
        )

        # Compute shape by doing one forward pass
        with torch.no_grad():
            n_flatten = net(
                torch.zeros(
                    [1, 2, self.context_size, self.horizontal_resolution], device=device
                )
            ).shape[1]

        super().__init__(space, n_flatten)

        # we cannot assign this directly to self.cnn before calling the super constructor
        self.net = net

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.net(observations)


class Compressor(nn.Module):
    def __init__(self, device: str = "cpu"):
        super().__init__()
        # WARNING : do not use inplace=True because it would modify the rollout buffer
        self.conv = nn.Conv1d(2, 64, kernel_size=7, stride=2, padding=3, device=device)
        self.dropout = nn.Dropout1d(0.3)
        self.pool = nn.MaxPool1d(kernel_size=3, stride=2, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x[:, :, 0]
        x = self.conv(x)
        x = self.dropout(x)
        x = self.pool(x)
        return x


class ResidualBlock(nn.Module):
    """
    basic block with a residual connection
    """

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        downsample: bool = False,
        device: str = "cpu",
    ):
        super().__init__()
        if downsample:
            stride = 2
            self.downsample = nn.Conv1d(
                in_channels, out_channels, kernel_size=1, stride=2, device=device
            )
        elif in_channels == out_channels:
            stride = 1
            self.downsample = nn.Identity()
        else:
            stride = 1
            self.downsample = nn.Conv1d(
                in_channels, out_channels, kernel_size=1, stride=1, device=device
            )

        self.bn1 = nn.BatchNorm1d(in_channels, device=device)
        self.conv1 = nn.Conv1d(
            in_channels,
            out_channels,
            kernel_size=3,
            stride=stride,
            padding=1,
            device=device,
        )

        self.bn2 = nn.BatchNorm1d(out_channels, device=device)
        self.conv2 = nn.Conv1d(
            out_channels, out_channels, kernel_size=3, padding=1, device=device
        )

        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout1d(0.4)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.bn1(x)
        y = self.relu(y)
        y = self.conv1(y)

        y = self.bn2(y)
        y = self.relu(y)
        y = self.dropout(y)
        y = self.conv2(y)

        y += self.downsample(x)

        return y
