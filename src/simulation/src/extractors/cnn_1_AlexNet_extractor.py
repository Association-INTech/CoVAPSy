import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class CNN1DAlexNetExtractor(BaseFeaturesExtractor):
    context_size = 5
    extra_context_channels = 2
    # speed_command, steering_command
    lidar_horizontal_resolution = 1080
    camera_horizontal_resolution = 1080
    n_sensors = 1
    wheelbase = 0.257  # meters
    dt = 0.032
    motion_feature_size = 4  # spd_cmd, steering_cmd, accel_cmd, yaw_rate

    # just an alias to avoid confusion because
    # the lidar and camera have the same resolution
    horizontal_resolution = 1080

    def __init__(
        self,
        space: spaces.Box,
        device: str = "cpu",
    ):
        cnn = nn.Sequential(
            # shape = [batch_size, 3, 1080]
            # we now have 3 channels : lidar, camera and savitzky-golay velocity of lidar
            nn.Conv1d(3, 64, kernel_size=7, stride=2, padding=3, device=device),
            nn.ReLU(),
            nn.MaxPool1d(3),
            # shape = [batch_size, 64, 180]
            nn.Conv1d(64, 64, kernel_size=3, padding="same", device=device),
            nn.ReLU(),
            nn.MaxPool1d(3),
            # shape = [batch_size, 64, 60]
            nn.Conv1d(64, 128, kernel_size=3, padding="same", device=device),
            nn.ReLU(),
            nn.AvgPool1d(2),
            # shape = [batch_size, 128, 30]
            nn.Conv1d(128, 128, kernel_size=3, padding="same", device=device),
            nn.ReLU(),
            nn.AvgPool1d(2),
            # shape = [batch_size, 128, 15]
            nn.Flatten(),
            # shape = [batch_size, 1920]
        )

        # Compute shape by doing one forward pass
        with torch.no_grad():
            n_flatten = cnn(
                torch.zeros([1, 3, self.horizontal_resolution], device=device)
            ).shape[1]

        super().__init__(space, n_flatten + self.motion_feature_size)

        # we cannot assign this directly to self.cnn before calling the super constructor
        self.net = cnn


    def forward(self, observations: dict[str, torch.Tensor]) -> torch.Tensor:
        sensors = observations["sensors"]    # shape: [B, 2, 5, 1080]
        controls = observations["controls"]  # shape: [B, 5, 2] -> (speed_cmd, steer_cmd)

        # Current lidar normalized to mean 0 and std 1, shape = [batch_size, 1080]
        
        lidar_all = sensors[:, 0, :, :]  # shape = [batch_size, 5, 1080]

        lidar_only = lidar_all[:, -1, :]  # shape = [batch_size, 1080]
        lidar_mean = lidar_only.mean(dim=1, keepdim=True)
        lidar_std = lidar_only.std(dim=1, keepdim=True)
        lidar_normalized = (lidar_only - lidar_mean) / (lidar_std + 1e-8)

        # Savitzky-Golay filter to compute velocity from lidar readings
        lidar_velocity = (
            2 * lidar_all[:, 4, :]
            + lidar_all[:, 3, :]
            - lidar_all[:, 1, :]
            - 2 * lidar_all[:, 0, :]
        ) / (10 * self.dt * 2.5)  # shape = [batch_size, 1080]
        lidar_velocity = lidar_velocity.clamp(-1.0, 1.0)

        # Current Camera scanline in {-1, 0, 1}
        cam_t = sensors[:, 1, -1, :]         # shape: [B, 1080]

        # Stack the 3 spatial channels -> [B, 3, 1080] and Conv1D + Flatten -> [B, 1920]
        spatial_in = torch.stack([lidar_normalized, lidar_velocity, cam_t], dim=1)
        flattened_cnn = self.net(spatial_in)  # shape: [B, 1920]

        # Extract the 4 Control Scalars (Normalized to [-1, 1])
        speed_cmd_mps = controls[:, :, 0] / 3.6          # [B, 5] in m/s (0.28 to 2.5)
        steer_cmd = controls[:, :, 1]                    # [B, 5] in rad (-0.4 to +0.4)

        cur_speed_norm = (speed_cmd_mps[:, -1:] / 2.5)   # [B, 1] in [0, 1]
        cur_steer_norm = (steer_cmd[:, -1:] / 0.4)       # [B, 1] in [-1, 1]
        
        accel_cmd = ((speed_cmd_mps[:, -1:] - speed_cmd_mps[:, -2:-1]) / self.dt).clamp(-4.0, 4.0) / 4.0
        yaw_rate = (speed_cmd_mps[:, -1:] * torch.tan(steer_cmd[:, -1:]) / self.wheelbase).clamp(-4.0, 4.0) / 4.0

        control_features = torch.cat(
            [cur_speed_norm, cur_steer_norm, accel_cmd, yaw_rate], dim=1
        )  # shape: [B, 4]

        # Concatenate right after Flatten() -> shape: [B, 1924]
        return torch.cat([flattened_cnn, control_features], dim=1)
