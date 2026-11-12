import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class TemporalResNetExtractor(BaseFeaturesExtractor):
    context_size = 128
    lidar_horizontal_resolution = 128
    camera_horizontal_resolution = 128
    camera_vertical_resolution = 1
    n_sensors = 1
    input_type = "LiDAR+Camera"
    horizontal_resolution = 128

    def __init__(
        self,
        space: spaces.Box,
        device: str = "cpu",
    ):
        net = nn.Sequential(
            Compressor(device),
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 128, downsample=True, device=device),
            ResidualBlock(128, 128, device=device),
            ResidualBlock(128, 256, downsample=True, device=device),
            ResidualBlock(256, 256, device=device),
            ResidualBlock(256, 512, downsample=True, device=device),
            ResidualBlock(512, 512, device=device),
            nn.AvgPool2d(4),
            nn.Flatten(),
        )

        with torch.no_grad():
            n_flatten = net(
                torch.zeros(
                    [1, 2, self.context_size, self.lidar_horizontal_resolution],
                    device=device,
                )
            ).shape[1]

        super().__init__(space, n_flatten)
        self.net = net

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.net(observations)


class Compressor(nn.Module):
    def __init__(self, device: str = "cpu"):
        super().__init__()
        self.conv = nn.Conv2d(2, 64, kernel_size=7, stride=2, padding=3, device=device)
        self.bn = nn.BatchNorm2d(64, device=device)
        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout2d(0.3)
        self.pool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.conv(x)
        x = self.bn(x)
        x = self.relu(x)
        x = self.dropout(x)
        return self.pool(x)


class ResidualBlock(nn.Module):
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
            self.downsample = nn.Conv2d(
                in_channels, out_channels, kernel_size=1, stride=2, device=device
            )
        elif in_channels == out_channels:
            stride = 1
            self.downsample = nn.Identity()
        else:
            stride = 1
            self.downsample = nn.Conv2d(
                in_channels, out_channels, kernel_size=1, stride=1, device=device
            )

        self.bn1 = nn.BatchNorm2d(in_channels, device=device)
        self.conv1 = nn.Conv2d(
            in_channels,
            out_channels,
            kernel_size=3,
            stride=stride,
            padding=1,
            device=device,
        )
        self.bn2 = nn.BatchNorm2d(out_channels, device=device)
        self.conv2 = nn.Conv2d(
            out_channels, out_channels, kernel_size=3, padding=1, device=device
        )
        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout2d(0.5)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.bn1(x)
        y = self.relu(y)
        y = self.conv1(y)
        y = self.bn2(y)
        y = self.relu(y)
        y = self.dropout(y)
        y = self.conv2(y)
        return y + self.downsample(x)
