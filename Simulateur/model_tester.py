import os
from stable_baselines3 import PPO
import torch.nn as nn
import torch
from config import *

from Simulateur.CNN1DExtractor import CNN1DExtractor
from Simulateur.TemporalResNetExtractor import TemporalResNetExtractor

import onnxruntime as ort


ExtractorClass = "CNN1DExtractor"
save_path = __file__.rsplit("/", 1)[0] + "/checkpoints/" + ExtractorClass + "/"

if not os.path.exists(save_path):
    raise FileNotFoundError(f"Directory {save_path} does not exist")

valid_files = [x for x in os.listdir(save_path) if x.rstrip(".zip").isnumeric()]

if not valid_files:
    raise FileNotFoundError(f"No valid files in {save_path}")

model_name = max(
    valid_files,
    key=lambda x : int(x.rstrip(".zip"))
)

ExctractorClass = TemporalResNetExtractor


ppo_args = dict(
    n_steps=2048,
    n_epochs=10,
    batch_size=512,
    learning_rate=3e-4,
    gamma=0.99,
    verbose=1,
    normalize_advantage=True,
    device=device
)
policy_kwargs = dict(
    features_extractor_class=ExtractorClass,
    features_extractor_kwargs=dict(
        context_size=context_size,
        lidar_horizontal_resolution=lidar_horizontal_resolution,
        camera_horizontal_resolution=camera_horizontal_resolution,
        device=device
    ),
    activation_fn=nn.ReLU,
    net_arch=[1024, 1024],
)

print(f"Loading model {save_path + model_name}")
sb_model = PPO.load(save_path + model_name, **ppo_args, policy_kwargs=policy_kwargs)

print(sb_model.policy)

model1 = nn.Sequential(
    sb_model.policy.features_extractor.net,
    sb_model.policy.mlp_extractor.policy_net,
    sb_model.policy.action_net
).to("cpu")
model1.eval()

session = ort.InferenceSession(save_path + model_name.rstrip(".zip") + ".onnx")
def model2(x):
    return session.run(None, {"input": x.cpu().numpy()})[0]

x = torch.randn(1000, 2, 128, 128)
loss_fn = nn.MSELoss()

with torch.no_grad():
    y1 = model1(x)
    y2 = model2(x)
    loss = loss_fn(y1, torch.tensor(y2))
    print(loss.item())
