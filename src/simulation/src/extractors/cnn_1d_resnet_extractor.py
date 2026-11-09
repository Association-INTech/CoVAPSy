import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class CNN1DResNetExtractor(BaseFeaturesExtractor):
    context_size = 1
    lidar_horizontal_resolution = 1024
    camera_horizontal_resolution = 1024
    n_sensors = 1
    wheelbase = 0.257  # meters
    dt = 0.025         # matches Webots basicTimeStep (25 ms)

    # just an alias to avoid confusion because
    # the lidar and camera have the same resolution
    horizontal_resolution = 1024

    def __init__(
        self,
        space: spaces.Dict,
    ):

        self.use_sg_velocity = self.context_size >= 5
        self.use_controls = "Speed_Cmd" in space.spaces and "Steer_Cmd" in space.spaces
        in_channels = (1 + int(self.use_sg_velocity)) + (self.camera_horizontal_resolution != 0)
        motion_dim = 4 if self.use_controls else 0

        """
        The idea behind ResNet sequence : 
        we want to increase context channel (in_channel) and decrease spatial resolution (1024)
        the goal is to have something like ~30 angular sector and ~60 context channel so
        the model essentially have enough weights to know that something is a wall and where it is
        so ideally we want the last shape to be [batch size, 64, 30]

        we usually want to keep the compute per layer to be constant
        that's why we want to either be dividing res either scaling context so we first need
        to upscale context to a reasonable value ~16 and downscale res to a reasonable value ~512
        """
        net = nn.Sequential(
            # [B, in_channels, 1024] -> [B, 16, 256]
            Compressor(in_channels, out_channels=16),
            # Stage 1: [B, 16, 256] -> [B, 32, 128]
            ResidualBlock(16, 32, downsample=True),
            ResidualBlock(32, 32),
            # Stage 2: [B, 32, 128] -> [B, 64, 64]
            ResidualBlock(32, 64, downsample=True),
            ResidualBlock(64, 64),
            # Stage 3: [B, 64, 64] -> [B, 64, 32] (Preserves 32 angular sectors!)
            ResidualBlock(64, 64, downsample=True),
            ResidualBlock(64, 64),
            nn.Flatten(),  # [B, 64 * 32 = 2048]
        )
        # Compute shape by doing one forward pass
        with torch.no_grad():
            n_flatten = net(
                torch.zeros(
                    [1, in_channels, self.horizontal_resolution]
                )
            ).shape[1]

        features_dim = 64 + (16 if self.use_controls else 0)

        super().__init__(space, features_dim)

        # we cannot assign this directly to self.cnn before calling the super constructor
        self.net = net
        self.spatial_proj = nn.Sequential(nn.Linear(n_flatten, 64), nn.ReLU())
        if self.use_controls:
            self.motion_proj = nn.Sequential(nn.Linear(motion_dim, 16), nn.ReLU())

    def forward(self, observations: dict[str, torch.Tensor]) -> torch.Tensor:
        lidar_all = observations["Lidar"]          # [B, T, W]
        lidar_t = lidar_all[:, -1, :]              # [B, W]
        lidar_mean = lidar_t.mean(dim=1, keepdim=True)
        lidar_std = lidar_t.std(dim=1, keepdim=True)
        lidar_norm = (lidar_t - lidar_mean) / (lidar_std + 1e-8)
        channels = [lidar_norm]
        if self.use_sg_velocity:
            lidar_vel = (
                2 * lidar_all[:, -1, :]
                + lidar_all[:, -2, :]
                - lidar_all[:, -4, :]
                - 2 * lidar_all[:, -5, :]
            ) / (10 * self.dt * 2.5)
            channels.append(lidar_vel.clamp(-1.0, 1.0))
        if "Cam" in observations and self.camera_horizontal_resolution != 0:
            channels.append(observations["Cam"][:, -1, :])
        spatial_in = torch.stack(channels, dim=1)  # [B, C, W]
        features = self.net(spatial_in)            # [B, 256]
        if self.use_controls:
            speed_cmd_mps = observations["Speed_Cmd"][:, :, 0] / 3.6  # [B, T]
            steer_cmd = observations["Steer_Cmd"][:, :, 0]            # [B, T]
            cur_speed_norm = speed_cmd_mps[:, -1:] / 2.5
            cur_steer_norm = steer_cmd[:, -1:] / 0.4
            if self.context_size >= 2:
                accel_cmd = ((speed_cmd_mps[:, -1:] - speed_cmd_mps[:, -2:-1]) / self.dt).clamp(-4.0, 4.0) / 4.0
            else:
                accel_cmd = torch.zeros_like(cur_speed_norm)
            yaw_rate = (speed_cmd_mps[:, -1:] * torch.tan(steer_cmd[:, -1:]) / self.wheelbase).clamp(-4.0, 4.0) / 4.0
            control_features = torch.cat([cur_speed_norm, cur_steer_norm, accel_cmd, yaw_rate], dim=1)
            
            # Features and control_features projection into a balanced embedding to increase control_features weight
            features = self.spatial_proj(features)
            control_features = self.motion_proj(control_features)
            # 80% features, 20% motion
            features = torch.cat([features, control_features], dim=1)
        return features


class Compressor(nn.Module):
    def __init__(self, in_channels: int = 2, out_channels: int = 16):
        super().__init__()
        self.conv = nn.Conv1d(in_channels, out_channels, kernel_size=5, stride=2, padding=2)
        self.pool = nn.MaxPool1d(kernel_size=3, stride=2, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.pool(self.conv(x))


class ResidualBlock(nn.Module):
    """
    basic block with a residual connection
    """

    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        downsample: bool = False
    ):
        super().__init__()
        stride = 2 if downsample else 1
        if downsample or in_channels != out_channels:
            self.downsample = nn.Conv1d(in_channels, out_channels, kernel_size=1, stride=stride)
        else:
            self.downsample = nn.Identity()

        self.gn1 = nn.GroupNorm(8, in_channels)
        self.conv1 = nn.Conv1d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1)
        self.gn2 = nn.GroupNorm(8, out_channels)
        self.conv2 = nn.Conv1d(out_channels, out_channels, kernel_size=3, stride=1, padding=1)
        self.relu = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.conv1(self.relu(self.gn1(x)))
        y = self.conv2(self.relu(self.gn2(y)))
        return y + self.downsample(x)
