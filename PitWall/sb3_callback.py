"""
CoVAPSy Pit-Wall — Stable-Baselines3 callback adapter.

Runs the TelemetryLogger in the *main* training process (works with
SubprocVecEnv because SB3 already brings actions/obs/rewards/infos back here).

    from sb3_callback import TelemetryCallback
    model.learn(total_timesteps=..., callback=TelemetryCallback("MemoryCNN_K5", n_checkpoints=31))

What it records per step and per env:
  * speed / steering   -> from ``info["speed"]`` / ``info["steering"]`` if your env provides them,
                          otherwise decoded from the discrete action bins (COMMANDED values)
  * min_lidar          -> min of the last LiDAR frame in the observation (ignores 0.0 = NaN fill)
  * checkpoint_id      -> ``info["checkpoint_id"]`` (needs the small supervisor patch in README), else -1
  * reward             -> SB3 reward
  * inference_ms       -> wall time of ``policy.forward`` (batched over all envs, CUDA-synchronised)
  * per-update metrics -> SB3 logger ``train/*`` values (entropy, approx_kl, clip_fraction, ...)
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

from telemetry_logger import TelemetryLogger

StepFn = Callable[[int, np.ndarray, np.ndarray, Dict[str, Any]], Dict[str, float]]

# CoVAPSy action decoding (controller_vehicle_driver.py)
_N_STEER, _N_SPEED = 16, 16
_STEER_LUT = np.linspace(-0.4, 0.4, _N_STEER)              # rad
_SPEED_LUT = np.linspace(1.0, 9.0, _N_SPEED) / 3.6         # km/h -> m/s


def covapsy_step_fn(env_id: int, action: np.ndarray, obs: Dict[str, np.ndarray] | np.ndarray, info: Dict[str, Any]) -> Dict[str, float]:
    """Default extractor for CoVAPSy's MultiDiscrete([16, 16]) action + Dict observation."""
    a = np.asarray(action).reshape(-1)
    speed = info.get("speed", _SPEED_LUT[int(a[1])])
    steering = info.get("steering", _STEER_LUT[int(a[0])])
    if isinstance(obs, dict):
        lidar_arr = np.asarray(obs["Lidar"])
        lidar = lidar_arr[-1, :] if lidar_arr.ndim == 2 else lidar_arr[0, -1, :]
    else:
        lidar_arr = np.asarray(obs)
        lidar = lidar_arr[-1, :] if lidar_arr.ndim == 2 else lidar_arr[0, -1, :]
    valid = lidar[lidar > 0.0]         # VehicleEnv maps NaN -> 0.0, don't count it as a wall
    min_lidar = float(valid.min()) if valid.size else float("nan")
    return {
        "speed": float(speed),
        "steering": float(steering),
        "min_lidar": min_lidar,
        "checkpoint_id": int(info.get("checkpoint_id", -1)),
    }


class TelemetryCallback(BaseCallback):
    TRAIN_KEYS = {
        "train/entropy_loss": "entropy_loss",
        "train/approx_kl": "approx_kl",
        "train/clip_fraction": "clip_fraction",
        "train/explained_variance": "explained_variance",
        "train/value_loss": "value_loss",
        "train/policy_gradient_loss": "policy_gradient_loss",
        "train/loss": "loss",
        "train/learning_rate": "learning_rate",
        "time/fps": "fps",
        # SAC / off-policy keys (picked up automatically if present)
        "train/ent_coef": "ent_coef",
        "train/actor_loss": "actor_loss",
        "train/critic_loss": "critic_loss",
    }

    def __init__(
        self,
        run_name: str,
        n_checkpoints: int,
        log_dir: str = "logs",
        step_fn: StepFn = covapsy_step_fn,
        time_inference: bool = True,
        crash_fn: Optional[Callable[[float, Dict[str, Any]], bool]] = None,
        meta: Optional[Dict[str, Any]] = None,
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.run_name = run_name
        self.n_checkpoints = n_checkpoints
        self.log_dir = log_dir
        self.step_fn = step_fn
        self.time_inference = time_inference
        # CoVAPSy: reward -0.5 <=> collision (controller_world_supervisor.py)
        self.crash_fn = crash_fn or (lambda r, info: bool(info.get("crashed", r < 0)))
        self.meta = meta or {}
        self._last_inf_ms = float("nan")
        self._total_steps = 0  # own counter: CoVAPSy re-calls learn() which resets num_timesteps
        self._env_cp: Optional[np.ndarray] = None  # fallback sector counter when info["checkpoint_id"] is absent
        self._local_ep: Optional[np.ndarray] = None  # per-env episode counter for synchronous cohort averaging
        self.tl: Optional[TelemetryLogger] = None

    # ------------------------------------------------------------------ setup
    def _on_training_start(self) -> None:
        if self.tl is not None:  # same callback reused across learn() calls -> keep streaming
            return
        n_envs = self.training_env.num_envs
        self._env_cp = np.zeros(n_envs, dtype=np.int64)
        self._local_ep = np.zeros(n_envs, dtype=np.int64)
        meta = {"arch": self.run_name, "algo": type(self.model).__name__, "n_envs": n_envs, **self.meta}
        self.tl = TelemetryLogger(self.run_name, self.n_checkpoints, self.log_dir, n_envs=n_envs, meta=meta)
        if self.time_inference:
            self._wrap_policy_forward()

    def _wrap_policy_forward(self) -> None:
        import torch

        policy = self.model.policy
        original = policy.forward
        on_cuda = str(policy.device).startswith("cuda")
        cb = self

        def timed_forward(*args, **kwargs):
            if on_cuda:
                torch.cuda.synchronize()
            t0 = time.perf_counter()
            out = original(*args, **kwargs)
            if on_cuda:
                torch.cuda.synchronize()
            cb._last_inf_ms = (time.perf_counter() - t0) * 1e3
            return out

        policy.forward = timed_forward  # instance attr shadows the method -> nn.Module.__call__ uses it

    # ------------------------------------------------------------------ per step
    def _on_step(self) -> bool:
        assert self.tl is not None
        actions = self.locals["actions"]
        rewards = self.locals["rewards"]
        dones = self.locals["dones"]
        infos = self.locals["infos"]
        new_obs = self.locals["new_obs"]
        self._total_steps += len(rewards)

        for i in range(len(rewards)):
            info = infos[i]
            r_i = float(rewards[i])
            # on episode end SB3 auto-resets: the real last obs is in terminal_observation
            if isinstance(new_obs, dict):
                if dones[i] and "terminal_observation" in info:
                    obs_i = info["terminal_observation"]
                else:
                    obs_i = {k: v[i] for k, v in new_obs.items()}
            else:
                obs_i = info.get("terminal_observation", new_obs[i]) if dones[i] else new_obs[i]
            fields = self.step_fn(i, actions[i], obs_i, info)
            if fields.get("checkpoint_id", -1) < 0 and self._env_cp is not None and self.n_checkpoints > 0:
                fields["checkpoint_id"] = int(self._env_cp[i])
                if r_i >= 1.0 - 1e-4:  # CoVAPSy checkpoint crossing reward >= 1.0
                    self._env_cp[i] = (self._env_cp[i] + 1) % self.n_checkpoints
            self.tl.log_step(
                reward=r_i,
                inference_ms=self._last_inf_ms,
                entropy=float(info.get("entropy", np.nan)),
                ts_gap=float(info.get("ts_l2_gap", np.nan)),
                wm_err=float(info.get("wm_recon_err", np.nan)),
                env_id=i,
                **fields,
            )
            if dones[i]:
                if self._env_cp is not None:
                    self._env_cp[i] = 0
                lep = 0
                if self._local_ep is not None:
                    self._local_ep[i] += 1
                    lep = int(self._local_ep[i])
                n_sims = int(self.meta.get("n_sims", 1) or 1)
                sim_rank = int(info.get("sim_rank", i % n_sims))
                vehicle_rank = int(info.get("vehicle_rank", i // n_sims))
                map_name = info.get("map") or self.meta.get("map", "piste0")
                self.tl.end_episode(
                    crashed=self.crash_fn(r_i, info),
                    env_id=i,
                    global_step=self._total_steps,
                    sim_rank=sim_rank,
                    vehicle_rank=vehicle_rank,
                    local_ep_idx=lep,
                    map_name=str(map_name) if map_name else None,
                )
        return True

    # ------------------------------------------------------------------ per update
    def _on_rollout_end(self) -> None:
        # SB3 order: collect_rollouts -> (this) -> dump_logs -> train().
        # name_to_value therefore still holds the train/* values of the previous update.
        assert self.tl is not None
        values = getattr(self.model.logger, "name_to_value", {})
        metrics = {short: values[k] for k, short in self.TRAIN_KEYS.items() if k in values}
        if "entropy_loss" in metrics:
            metrics["entropy"] = -metrics["entropy_loss"]
        if metrics:
            self.tl.log_train(self._total_steps, **metrics)

    # No _on_training_end close(): the logger lives across learn() calls and is
    # flushed/closed automatically at interpreter exit (atexit in TelemetryLogger).
