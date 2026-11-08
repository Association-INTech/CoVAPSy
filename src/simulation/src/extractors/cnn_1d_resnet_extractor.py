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
        device: str = "cpu",
    ):

        self.use_sg_velocity = self.context_size >= 5
        self.use_controls = "Speed_Cmd" in space.spaces and "Steer_Cmd" in space.spaces
        in_channels = (1 + int(self.use_sg_velocity)) + (self.camera_horizontal_resolution != 0)
        motion_dim = 4 if self.use_controls else 0

        net = nn.Sequential(
            # shape = [batch_size, in_channels, 1024]
            Compressor(in_channels, device=device),
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
                    [1, in_channels, self.horizontal_resolution], device=device
                )
            ).shape[1]

        super().__init__(space, n_flatten + motion_dim)

        # we cannot assign this directly to self.cnn before calling the super constructor
        self.net = net

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
            features = torch.cat([features, control_features], dim=1)
        return features


class Compressor(nn.Module):
    def __init__(self, in_channels: int = 2, device: str = "cpu"):
        super().__init__()
        # WARNING : do not use inplace=True because it would modify the rollout buffer
        self.conv = nn.Conv1d(in_channels, 64, kernel_size=7, stride=2, padding=3, device=device)
        self.dropout = nn.Dropout1d(0.3)
        self.pool = nn.MaxPool1d(kernel_size=3, stride=2, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
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
