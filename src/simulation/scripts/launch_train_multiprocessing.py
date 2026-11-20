import os
import shutil
import sys
from logging import DEBUG
from pathlib import Path

# Pin to CPU 0 during module imports so PyTorch's C++ ApproximateClock
# cannot migrate across desynchronized AMD TSC cores.
_ORIGINAL_AFFINITY = os.sched_getaffinity(0) if hasattr(os, "sched_getaffinity") else None
if _ORIGINAL_AFFINITY:
    os.sched_setaffinity(0, {min(_ORIGINAL_AFFINITY)})

# Make the project source root importable when this script is run directly.
SRC_DIR = Path(__file__).resolve().parents[2]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from simulation import config as c  #
from simulation import VehicleEnv  #

if __name__ == "__main__":
    print("[1/4] Importing PyTorch, Stable-Baselines3 & PitWall...", flush=True)
    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.vec_env import SubprocVecEnv  #[cite: 1]
    from utils import onnx_utils  #[cite: 1]

    # Restore full multi-core CPU affinity now that torch is initialized
    if _ORIGINAL_AFFINITY:
        os.sched_setaffinity(0, _ORIGINAL_AFFINITY)

    PITWALL_DIR = Path(__file__).resolve().parents[3] / "PitWall"  #[cite: 1]
    sys.path.insert(0, str(PITWALL_DIR))  #[cite: 1]
    from sb3_callback import TelemetryCallback  #[cite: 1]

    shutil.rmtree(c.FIFO_DIR, ignore_errors=True)  #[cite: 1]
    os.makedirs(c.FIFO_DIR, exist_ok=True)  #[cite: 1]

    model_name = f"{c.ExtractorClass.__name__}"  #[cite: 1]
    if c.context_size > 1:  #[cite: 1]
        model_name += f"context_{c.context_size}"  #[cite: 1]
    algo_name = "SAC" if c.continuous_actions else "PPO"
    # to change if we go back to no context or no motion, but for now we are using context5 and motion[cite: 1]
    telemetry_cb = TelemetryCallback(
        run_name=f"{algo_name}_{model_name}",
        n_checkpoints=31,
        log_dir=str(PITWALL_DIR / "logs"),
        meta={
            "map": f"track{c.map_id}",
            "lidar_res": c.lidar_horizontal_resolution,
            "n_sims": c.n_simulations,
            "n_vehicles": c.n_vehicles,
        },
    )  #[cite: 1]

    print("[2/4] Launching Webots (--no-rendering) & waiting for pipes...", flush=True)
    envs = SubprocVecEnv(
        [
            lambda sr=simulation_rank, vr=vehicle_rank: VehicleEnv(sr, vr)
            for vehicle_rank in range(c.n_vehicles)
            for simulation_rank in range(c.n_simulations)
        ],
        start_method="spawn",
    )  #[cite: 1]

    print(f"[3/4] Initializing {algo_name} model...", flush=True)
    save_path = c.save_dir / "checkpoints" / f"{algo_name}_{model_name}"  #[cite: 1]
    save_path.mkdir(parents=True, exist_ok=True)  #[cite: 1]

    valid_files = [x for x in save_path.iterdir() if x.name.rstrip(".zip").isnumeric()]  #[cite: 1]

    if valid_files:  #[cite: 1]
        model_path = max(valid_files, key=lambda x: int(x.name.rstrip(".zip")))  #[cite: 1]
        print(f"Loading model {model_path.name}", flush=True)
        model = (
            SAC.load(model_path, envs, **c.sac_args, policy_kwargs=c.policy_kwargs)
            if c.continuous_actions
            else PPO.load(model_path, envs, **c.ppo_args, policy_kwargs=c.policy_kwargs)
        )  #[cite: 1]
        i = int(model_path.name.rstrip(".zip")) + 1  #[cite: 1]
        print(f"Model found, loading {model_path}", flush=True)
    else:  #[cite: 1]
        model = (
            SAC("MultiInputPolicy", envs, **c.sac_args, policy_kwargs=c.policy_kwargs)
            if c.continuous_actions
            else PPO("MultiInputPolicy", envs, **c.ppo_args, policy_kwargs=c.policy_kwargs)
        )  #[cite: 1]
        i = 0  #[cite: 1]
        print("Model not found, creating a new one", flush=True)

    print("model hyper parameters:")
    print(f"{model.learning_rate=}")
    print(f"{model.gamma=}")
    print(f"{model.verbose=}")
    if c.continuous_actions:
        print(f"{model.buffer_size=}")
        print(f"{model.learning_starts=}")
        print(f"{model.train_freq=}")
    else:
        print(f"{model.n_steps=}")
        print(f"{model.n_epochs=}")
    print(f"{model.batch_size=}")
    print(f"{model.device=}", flush=True)

    while True:
        print(f"[4/4] Iteration {i}: exporting ONNX & starting {algo_name} learn()...", flush=True)
        onnx_utils.export_onnx(
            model,
            str(c.save_dir / f"model_{c.ExtractorClass.__name__}.onnx"),
        )  #[cite: 1]
        onnx_utils.test_onnx(model)  #[cite: 1]

        if c.LOG_LEVEL <= DEBUG:  #[cite: 1]
            from utils import PlotModelIO  #[cite: 1]

            model.learn(
                total_timesteps=c.total_timesteps,
                progress_bar=False,
                callback=[PlotModelIO(), telemetry_cb],
                reset_num_timesteps=(i == 0),
            )  #[cite: 1]
        else:  #[cite: 1]
            model.learn(
                total_timesteps=c.total_timesteps,
                progress_bar=True,
                callback=telemetry_cb,
                reset_num_timesteps=(i == 0),
            )  #[cite: 1]

        print("iteration over", flush=True)
        model.save(save_path / str(i))  #[cite: 1]
        i += 1  #[cite: 1]