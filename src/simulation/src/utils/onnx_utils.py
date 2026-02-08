import os

import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn
from stable_baselines3.common.on_policy_algorithm import OnPolicyAlgorithm

from simulation import config as c


def get_torch_model(sb_model):
    return nn.Sequential(
        sb_model.policy.features_extractor.net,
        sb_model.policy.mlp_extractor.policy_net,
        sb_model.policy.action_net,
    ).to("cpu")


def export_onnx(sb_model: OnPolicyAlgorithm, path: str):
    device = sb_model.policy.device
    torch_model = get_torch_model(sb_model)
    torch_model.eval()

    example_input = torch.randn(1, 2, c.context_size, c.lidar_horizontal_resolution)

    with torch.no_grad():
        torch.onnx.export(
            torch_model,
            (example_input,),
            path,
            input_names=["input"],
            dynamo=True,
            output_names=["output"],
            dynamic_shapes={"input": {0: "batch_size"}},
        )

    torch_model.to(device)
    sb_model.policy.to(device)
    sb_model.policy.train()


def run_onnx_model(session: ort.InferenceSession, x: np.ndarray):
    return session.run(None, {"input": x})[0]


def test_onnx(model: OnPolicyAlgorithm):
    device = model.policy.device
    model.policy.eval()
    true_model = get_torch_model(model)

    loss_fn = nn.MSELoss()
    x = torch.randn(1000, 2, c.context_size, c.lidar_horizontal_resolution)

    try:
        class_name = model.policy.features_extractor.__class__.__name__
        model_path = os.path.expanduser(f"~/.cache/autotech/model_{class_name}.onnx")

        session = ort.InferenceSession(model_path)
    except Exception as e:
        print(f"Error loading ONNX model: {e}")
        return

    with torch.no_grad():
        y_true_test = true_model(x)

        true_model.train()
        y_true_train = true_model(x)
        true_model.eval()

        y_onnx = run_onnx_model(session, x.cpu().numpy())

        loss_test = loss_fn(y_true_test, torch.tensor(y_onnx))
        loss_train = loss_fn(y_true_train, torch.tensor(y_onnx))
        print(f"onnx_test={loss_test}")
        print(f"onnx_train={loss_train}")

    true_model.to(device)
    model.policy.to(device)
    model.policy.train()
