import os
from stable_baselines3 import PPO
import torch.nn as nn
import torch
from config import *

from CNN1DExtractor import CNN1DExtractor
from TemporalResNetExtractor import TemporalResNetExtractor

import onnxruntime as ort


def get_true_model(model):
    return nn.Sequential(
        model.policy.features_extractor.net,
        model.policy.mlp_extractor.policy_net,
        model.policy.action_net
    ).to("cpu")


def export_onnx(model):
    model.policy.eval()
    device = model.policy.device
    true_model = get_true_model(model)
    x = torch.randn(1, 2, context_size, lidar_horizontal_resolution)

    with torch.no_grad():
        torch.onnx.export(
            true_model,
            x,
            "model.onnx",
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}}
        )

    true_model.to(device)
    model.policy.to(device)
    model.policy.train()


def test_onnx(model):
    device = model.policy.device
    model.policy.eval()
    true_model = get_true_model(model)

    try:
        session = ort.InferenceSession("model.onnx")
    except Exception as e:
        print(f"Error loading ONNX model: {e}")
        return

    def model_onnx(x):
        return session.run(None, {"input": x.cpu().numpy()})[0]

    loss_fn = nn.MSELoss()
    x = torch.randn(1000, 2, context_size, lidar_horizontal_resolution)

    with torch.no_grad():
        y_true_test = true_model(x)

        true_model.train()
        y_true_train = true_model(x)
        true_model.eval()

        y_onnx = model_onnx(x)

        loss_test = loss_fn(y_true_test, torch.tensor(y_onnx))
        loss_train = loss_fn(y_true_train, torch.tensor(y_onnx))
        print(f"onnx_test={loss_test}")
        print(f"onnx_train={loss_train}")

    true_model.to(device)
    model.policy.to(device)
    model.policy.train()

