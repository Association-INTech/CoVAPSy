import torch
import torch.nn as nn
import torch.nn.functional as F
from gymnasium import spaces

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor




# class ChannelDependentDropout2d(nn.Module):
#     def __init__(self, p: list[float], inplace: bool = False):
#         super().__init__()

#         self.dropouts = nn.ModuleList([
#             nn.Dropout2d(p=q, inplace=inplace) for q in p
#         ])

#     def forward(self, x: torch.Tensor) -> torch.Tensor:
#         if x.dim() != 4:
#             raise ValueError("input tensor must have 4 dimensions")
#         if x.shape[1] != len(self.dropouts):
#             raise ValueError(f"input tensor has {x.shape[1]} channels, expected {len(self.dropouts)}")

#         return torch.cat(
#             [drop(x[:, i, None, :, :]) * (1-drop.p) for i, drop in enumerate(self.dropouts)],
#             dim=1
#         )


class Compressor(nn.Module):
    def __init__(self, device: str = "cpu"):
        super().__init__()
        # WARNING : do not use inplace=True because it would modify the rollout buffer
        # self.input_dropout = ChannelDependentDropout2d([0.001, 0.8])
        self.conv = nn.Conv2d(2, 64, kernel_size=7, stride=2, padding=3, device=device)
        self.bn = nn.BatchNorm2d(64, device=device)
        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout2d(0.3)
        self.pool = nn.MaxPool2d(kernel_size=3, stride=2, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x = self.input_dropout(x)
        x = self.conv(x)
        x = self.bn(x)
        x = self.relu(x)
        x = self.dropout(x)
        x = self.pool(x)
        return x




class ResidualBlock(nn.Module):
    """
    basic block with a residual connection
    """

    def __init__(self, in_channels: int, out_channels: int, downsample: bool = False, device: str = "cpu"):
        super().__init__()
        if downsample:
            stride = 2
            self.downsample = nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=2, device=device)
        else:
            stride = 1
            self.downsample = nn.Identity()

        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, device=device)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1, device=device)
        self.bn1 = nn.BatchNorm2d(out_channels, device=device)
        self.bn2 = nn.BatchNorm2d(out_channels, device=device)
        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout2d(0.3)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.conv1(x)
        y = self.bn1(y)
        y = self.relu(y)
        y = self.dropout(y)

        y = self.conv2(y)
        y = self.bn2(y)
        y = self.relu(y)
        y = self.dropout(y)

        x = self.downsample(x)
        return x + y




class TemporalResNetExtractor(BaseFeaturesExtractor):
    def __init__(self, space: spaces.Box, context_size: int, lidar_horizontal_resolution: int, camera_horizontal_resolution: int, device: str = "cpu"):
        if (context_size, lidar_horizontal_resolution, camera_horizontal_resolution) != 3*(128,):
            raise ValueError("context_size must be 128 for TemporalResNetExtractor")

        self.lidar_horizontal_resolution = lidar_horizontal_resolution
        self.camera_horizontal_resolution = camera_horizontal_resolution

        net = nn.Sequential(
            Compressor(device),
            # shape = [batch_size, 64, 32, 32]

            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 64, device=device),
            ResidualBlock(64, 64, device=device),
            # shape = [batch_size, 64, 32, 32]

            ResidualBlock(64, 128, downsample=True, device=device),
            ResidualBlock(128, 128, device=device),
            ResidualBlock(128, 128, device=device),
            # shape = [batch_size, 128, 16, 16]

            ResidualBlock(128, 256, downsample=True, device=device),
            ResidualBlock(256, 256, device=device),
            ResidualBlock(256, 256, device=device),
            # shape = [batch_size, 256, 8, 8]

            nn.AvgPool2d(8),
            # shape = [batch_size, 256, 1, 1]

            nn.Flatten(),
            # shape = [batch_size, 256]
        )

        # Compute shape by doing one forward pass
        with torch.no_grad():
            n_flatten = net(
                torch.zeros([1, 2, 128, 128], dtype=torch.float32, device=device)
            ).shape[1]
        print("n_flatten: ", n_flatten)
        super().__init__(space, n_flatten)

        # we cannot assign this directly to self.cnn before calling the super constructor
        self.net = net


    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        extracted = self.net(observations)

        return extracted
