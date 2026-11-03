import matplotlib.pyplot as plt
import numpy as np
import torch
from stable_baselines3.common.callbacks import BaseCallback

from simulation import config as c
from utils.camera import camera_color_profile

steering_true = (c.n_actions_steering + 1) // 2
SENSOR_PLOT_HEIGHT = 64


class PlotModelIO(BaseCallback):
    """
    A stable_baselines3 callback used to plot live visualizations of model inputs and outputs.
    """

    def __init__(self, verbose=0):
        super().__init__(verbose)
        n_plots = 5 if c.input_type == "image" else 4
        self.fig, self.ax = plt.subplots(n_plots, 1, figsize=(10, 2 * n_plots))

        # Steering bars
        self.steering_bars = self.ax[0].bar(
            range(c.n_actions_steering), np.zeros(c.n_actions_steering), color="blue"
        )
        self.steering_avg = [
            self.ax[0].plot(
                [0, 0], [0, 1], color=(i / 3, 1 - i / 3, 0), label="Average"
            )[0]
            for i in range(4)
        ]
        self.ax[0].set_ylim(0, 1)  # Probabilities range from 0 to 1
        self.ax[0].set_title("Steering Action Probabilities")

        # Speed bars
        self.speed_bars = self.ax[1].bar(
            range(c.n_actions_speed), np.zeros(c.n_actions_speed), color="blue"
        )
        self.speed_avg = self.ax[1].plot([0, 0], [0, 1], color="red", label="Average")[
            0
        ]
        self.ax[1].set_ylim(0, 1)  # Probabilities range from 0 to 1
        self.ax[1].set_title("Speed Action Probabilities")

        if c.input_type != "image" :

            # LiDAR img
            self.lidar_img = self.ax[2].imshow(
                np.zeros((SENSOR_PLOT_HEIGHT, c.lidar_horizontal_resolution)),
                cmap="gray",
                vmin=0.0,
                vmax=1.0,
                aspect="auto",
            )
            self.ax[2].set_title("LiDAR Image")

        # Camera img
        if c.camera_horizontal_resolution:
            if c.input_type == "image":
                self.camera_img = self.ax[3].imshow(
                    np.zeros(
                        (
                            c.camera_vertical_resolution,
                            c.camera_horizontal_resolution,
                            3,
                        )
                    ),
                    aspect="auto",
                )
            else:
                self.camera_img = self.ax[3].imshow(
                    np.zeros(
                        (SENSOR_PLOT_HEIGHT, c.camera_horizontal_resolution)
                    ),
                    cmap="RdYlGn",
                    vmin=-1.0,
                    vmax=1.0,
                    aspect="auto",
                )
            self.ax[3].set_title("Camera Image")

        if c.input_type == "image":
            self.color_profile_bars = self.ax[4].bar(
                range(c.camera_horizontal_resolution),
                np.zeros(c.camera_horizontal_resolution),
                color="gray",
            )
            self.ax[4].axhline(0, color="black", linewidth=0.8)
            self.ax[4].set_ylim(-1, 1)
            self.ax[4].set_xlim(-0.5, c.camera_horizontal_resolution - 0.5)
            self.ax[4].set_title("Camera Color Profile (green +1 / red -1)")
            self.ax[4].set_xlabel("Image column")

    def _on_step(self) -> bool:
        global steering_true

        obs = self.locals["obs_tensor"].clone().detach()

        if c.input_type != "image":
            lidar_data = (
                obs[0, 0, :, :].cpu().numpy()
                if c.context_size > 1
                else obs[0, 0, :].cpu().numpy()
            )
            if c.context_size > 1:
                self.lidar_img.set_data(lidar_data)
            else:
                lidar_strip = np.repeat(
                    lidar_data[np.newaxis, :], SENSOR_PLOT_HEIGHT, axis=0
                )
                self.lidar_img.set_data(lidar_strip)

            if c.camera_horizontal_resolution:
                camera_data = (
                    obs[0, 1, :, :].cpu().numpy()
                    if c.context_size > 1
                    else obs[0, 1, :].cpu().numpy()
                )
                if c.context_size > 1:
                    self.camera_img.set_data(camera_data)
                else:
                    camera_strip = np.repeat(
                        camera_data[np.newaxis, :], SENSOR_PLOT_HEIGHT, axis=0
                    )
                    self.camera_img.set_data(camera_strip)
        elif c.input_type == "image":
            camera_data = obs[0].permute(1, 2, 0).cpu().numpy()
            self.camera_img.set_data(camera_data)
            profile = camera_color_profile(
                np.clip(camera_data * 255, 0, 255).astype(np.uint8)
            )
            for bar, value in zip(self.color_profile_bars, profile):
                bar.set_height(value)
                bar.set_color("green" if value > 0 else "red" if value < 0 else "gray")
        with torch.no_grad():
            latent = self.model.policy.features_extractor(obs)
            extracted = self.model.policy.mlp_extractor.policy_net(latent)
            output = self.model.policy.action_net(extracted)[0]

            # Test different temperatures
            T = [0.4, 0.6, 0.8, 1.0]
            steer_probs = torch.softmax(output[: c.n_actions_steering], dim=-1).to(
                "cpu"
            )
            speed_probs = torch.softmax(output[c.n_actions_steering :], dim=-1).to(
                "cpu"
            )

            # Update the probabilities
            for i, bar in enumerate(self.steering_bars):
                bar.set_height(steer_probs[i].item())

            for i in range(4):
                steer_probs_T = torch.softmax(
                    output[: c.n_actions_steering] / T[i], dim=-1
                ).to("cpu")
                steering_avg = (
                    (steer_probs_T / steer_probs_T.sum() * torch.arange(16))
                    .sum()
                    .item()
                )
                self.steering_avg[i].set_xdata([steering_avg, steering_avg])

            for i, bar in enumerate(self.speed_bars):
                bar.set_height(speed_probs[i].item())

            speed_avg = (speed_probs * torch.arange(c.n_actions_speed)).sum().item()
            self.speed_avg.set_xdata([speed_avg, speed_avg])

        plt.draw()
        plt.pause(1e-8)

        return True  # Continue training
