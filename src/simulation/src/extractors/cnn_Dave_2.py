import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class DAVE2Extractor(BaseFeaturesExtractor):
    """
    DAVE-2 (NVIDIA end-to-end self-driving CNN), adapted to 1D sensor input.
    Bojarski et al., "End to End Learning for Self-Driving Cars", 2016.

    The layer widths, kernel sizes and strides follow the paper (24/36/48 with
    5-wide strided kernels, then 64/64 with 3-wide kernels, then dense
    100 -> 50 -> 10), but convolutions are 1D over the horizontal axis.

    The final steering neuron of the paper is NOT included: the 10-dimensional
    output is the feature vector, and the SB3 policy / value heads sit on top.
    """

    # Read by config.py on the class itself (before any instance exists) to
    # build the environment's observation space, same as in the template.
    lidar_horizontal_resolution = 1024
    camera_horizontal_resolution = 1024
    camera_vertical_resolution = 1
    n_sensors = 1

    # just an alias to avoid confusion because
    # the lidar and camera have the same resolution
    horizontal_resolution = 1024

    features_dim_out = 10

    def __init__(
        self,
        space: spaces.Box,
        conv_dropout: float = 0.2,
        fc_dropout: float = 0.3,
        device: str = "cpu",
    ):
        n_sensors, horizontal_resolution = space.shape
        in_channels = n_sensors

        conv = nn.Sequential(
            # shape = [batch_size, n_sensors, 1024]
            nn.Conv1d(in_channels, 24, kernel_size=5, stride=2, device=device),
            nn.ReLU(),
            # shape = [batch_size, 24, 510]
            nn.Conv1d(24, 36, kernel_size=5, stride=2, device=device),
            nn.ReLU(),
            nn.Dropout1d(conv_dropout),
            # shape = [batch_size, 36, 253]
            nn.Conv1d(36, 48, kernel_size=5, stride=2, device=device),
            nn.ReLU(),
            nn.Dropout1d(conv_dropout),
            # shape = [batch_size, 48, 125]
            nn.Conv1d(48, 64, kernel_size=3, device=device),
            nn.ReLU(),
            nn.Dropout1d(conv_dropout),
            # shape = [batch_size, 64, 123]
            nn.Conv1d(64, 64, kernel_size=3, device=device),
            nn.ReLU(),
            nn.Dropout1d(conv_dropout),
            # shape = [batch_size, 64, 121]
            nn.Flatten(),
            # shape = [batch_size, 64 * 121 = 7744]
        )

        # Compute the flattened size by doing one forward pass
        # (eval mode so dropout does not interfere)
        conv.eval()
        with torch.no_grad():
            n_flatten = conv(
                torch.zeros([1, in_channels, horizontal_resolution], device=device)
            ).shape[1]

        fc = nn.Sequential(
            nn.Linear(n_flatten, 100, device=device),
            nn.ReLU(),
            nn.Dropout(fc_dropout),
            # shape = [batch_size, 100]
            nn.Linear(100, 50, device=device),
            nn.ReLU(),
            nn.Dropout(fc_dropout),
            # shape = [batch_size, 50]
            nn.Linear(50, self.features_dim_out, device=device),
            nn.ReLU(),
            # shape = [batch_size, 10]
        )

        super().__init__(space, self.features_dim_out)

        # we cannot assign these directly to self before calling the super constructor
        self.net = nn.Sequential(conv, fc)
        self.net.train()

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        return self.net(observations)


# Usage with SB3:
#
# policy_kwargs = dict(
#     features_extractor_class=DAVE2Extractor,
#     features_extractor_kwargs=dict(conv_dropout=0.2, fc_dropout=0.3),
# )
# model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs)