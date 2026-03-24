import os
import sys
from pathlib import Path

import numpy as np
import onnxruntime as ort

import simulation.config as c
from extractors import (  # noqa: F401
    CNN1DResNetExtractor,
    TemporalResNetExtractor,
)
from simulation import VehicleEnv
from utils import run_onnx_model

ONNX_MODEL_PATH = c.save_dir / f"model_{c.ExtractorClass}.onnx"


def init_onnx_runtime_session(onnx_path: Path) -> ort.InferenceSession:
    if not os.path.exists(onnx_path):
        raise FileNotFoundError(
            f"The ONNX file could not be found at: {onnx_path}. Please export it first."
        )
    return ort.InferenceSession(onnx_path)


if __name__ == "__main__":
    if not os.path.exists("/tmp/autotech/"):
        os.mkdir("/tmp/autotech/")

    os.system('if [ -n "$(ls /tmp/autotech)" ]; then rm /tmp/autotech/*; fi')

    # Starting the ONNX session
    try:
        ort_session = init_onnx_runtime_session(ONNX_MODEL_PATH)
        input_name = ort_session.get_inputs()[0].name
        output_name = ort_session.get_outputs()[0].name
        print(f"ONNX model loaded from {ONNX_MODEL_PATH}")
        print(f"Input Name: {input_name}, Output Name: {output_name}")
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    env = VehicleEnv(0, 0)
    obs, _ = env.reset()

    print("Starting simulation in inference mode...")

    step_count = 0

    while True:
        raw_action = run_onnx_model(ort_session, obs[None])
        logits = np.array(raw_action).flatten()

        steer_logits = logits[: c.n_actions_steering]
        speed_logits = logits[c.n_actions_steering :]

        action_steer = np.argmax(steer_logits)
        action_speed = np.argmax(speed_logits)

        action = np.array([action_steer, action_speed], dtype=np.int64)

        next_obs, reward, done, truncated, info = env.step(action)

        step_count += 1

        if done:
            print(f"Episode(s) finished after {step_count} steps.")
            step_count = 0

            fresh_frame = next_obs[:, -1:]
            obs, _ = env.reset()
            env.context[:, -1:] = fresh_frame
            obs = env.context
        else:
            obs = next_obs
