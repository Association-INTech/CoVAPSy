import os

import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn
from stable_baselines3.common.on_policy_algorithm import OnPolicyAlgorithm

from simulation import config as c


class DictOnnxWrapper(nn.Module):
    def __init__(self, sb_model, keys: list[str]):
        super().__init__()
        self.keys = keys
        self.extractor = sb_model.policy.features_extractor
        self.policy_net = sb_model.policy.mlp_extractor.policy_net
        self.action_net = sb_model.policy.action_net

    def forward(self, *inputs: torch.Tensor) -> torch.Tensor:
        obs_dict = {k: t for k, t in zip(self.keys, inputs)}
        features = self.extractor(obs_dict)
        latent_pi = self.policy_net(features)
        return self.action_net(latent_pi)


def get_torch_model(sb_model):
    return DictOnnxWrapper(sb_model, list(c.active_obs)).to("cpu")


def export_onnx(sb_model: OnPolicyAlgorithm, path: str):
    device = sb_model.policy.device
    torch_model = get_torch_model(sb_model)
    torch_model.eval()

    example_inputs = tuple(
        torch.randn(1, c.context_size, c.OBS_SPECS[k]) for k in c.active_obs
    )

    with torch.no_grad():
        torch.onnx.export(
            torch_model,
            example_inputs,
            path,
            input_names=list(c.active_obs),
            output_names=["output"],
            dynamic_axes={
                **{k: {0: "batch_size"} for k in c.active_obs},
                "output": {0: "batch_size"},
            },
        )

    torch_model.to(device)
    sb_model.policy.to(device)
    sb_model.policy.train()


def run_onnx_model(session: ort.InferenceSession, x: dict[str, np.ndarray] | np.ndarray):
    if isinstance(x, dict):
        feed = {
            k: (v[None] if v.ndim == 2 else v).astype(np.float32, copy=False)
            for k, v in x.items()
        }
        return session.run(None, feed)[0]
    return session.run(None, {"input": x})[0]


def test_onnx(model: OnPolicyAlgorithm):
    device = model.policy.device
    model.policy.eval()
    true_model = get_torch_model(model)

    loss_fn = nn.MSELoss()
    x_tensors = tuple(
        torch.randn(100, c.context_size, c.OBS_SPECS[k]) for k in c.active_obs
    )
    x_dict_np = {
        k: t.cpu().numpy() for k, t in zip(c.active_obs, x_tensors)
    }

    try:
        class_name = model.policy.features_extractor.__class__.__name__
        model_path = c.save_dir / f"model_{class_name}.onnx"
        session = ort.InferenceSession(model_path)
    except Exception as e:
        print(f"Error loading ONNX model: {e}")
        return

    with torch.no_grad():
        y_true_test = true_model(*x_tensors)

        true_model.train()
        y_true_train = true_model(*x_tensors)
        true_model.eval()

        y_onnx = run_onnx_model(session, x_dict_np)

        loss_test = loss_fn(y_true_test, torch.tensor(y_onnx))
        loss_train = loss_fn(y_true_train, torch.tensor(y_onnx))
        print(f"onnx_test={loss_test}")
        print(f"onnx_train={loss_train}")

    true_model.to(device)
    model.policy.to(device)
    model.policy.train()
