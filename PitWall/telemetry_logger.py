"""
CoVAPSy Pit-Wall — zero-blocking telemetry producer.

Drop-in logger for headless (``--no-rendering``) Webots RL runs. It buffers
step-level data in pre-allocated NumPy arrays and, at the end of each episode,
hands a *copy* of the buffer to a background thread that summarises it and
appends ONE JSON line to ``<log_dir>/<run_name>.jsonl``.

Hot-path cost of ``log_step`` = a handful of NumPy scalar assignments (~µs).
Summarisation, JSON encoding and disk I/O never run on the training thread.

Usage (inside a Gymnasium env, one logger per process)::

    from telemetry_logger import TelemetryLogger

    tl = TelemetryLogger("MemoryCNN_K5", n_checkpoints=31, log_dir="logs",
                         meta={"arch": "MemoryCNN_K5", "map": "piste0"})
    ...
    tl.log_step(speed=v_mps, steering=delta_rad, min_lidar=lidar.min(),
                checkpoint_id=next_cp, reward=r, inference_ms=dt_ms)
    if done:
        tl.end_episode(crashed=(r < 0), global_step=step)
    ...
    tl.close()

Record schema (one per line, see README.md for every field)::

    {"type": "episode", "run_name": ..., "episode": 12, "global_step": 40960,
     "steps": 812, "mean_speed": 1.83, "crashed": true, "crash_checkpoint": 17,
     "steer_osc": 0.021, "inference_ms": {"p50": .., "p95": ..}, ...
     "per_checkpoint": {"steps": [...], "mean_speed": [...], "min_lidar": [...]}}

    {"type": "train", "run_name": ..., "global_step": 40960,
     "entropy": 4.1, "approx_kl": 0.012, ...}
"""

from __future__ import annotations

import atexit
import json
import math
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

SCHEMA_VERSION = 1
DEFAULT_DT = 0.025  # Webots basicTimeStep in CoVAPSy (25 ms -> 40 Hz)

# Column layout of the per-step buffer (float64 for simplicity / precision)
_COLS = (
    "speed",          # m/s (physical if available, else commanded)
    "steering",       # rad (applied steering angle)
    "min_lidar",      # m   (closest LiDAR return = wall clearance)
    "checkpoint_id",  # int (sector id = index of the NEXT checkpoint), -1 = unknown
    "reward",         # step reward
    "inference_ms",   # policy forward latency, NaN if unknown
    "entropy",        # optional per-step policy entropy, NaN if unknown
    "ts_gap",         # optional ||pi_S - pi_T||_2 (teacher-student), NaN if unknown
    "wm_err",         # optional world-model reconstruction error, NaN if unknown
)
_IDX = {name: i for i, name in enumerate(_COLS)}
_NAN = float("nan")


def _f(x: Any, nd: int = 4) -> Optional[float]:
    """JSON-safe float: NaN/inf -> None, rounded."""
    if x is None:
        return None
    x = float(x)
    if not math.isfinite(x):
        return None
    return round(x, nd)


def _flist(a: np.ndarray, nd: int = 4) -> list:
    return [_f(v, nd) for v in a]


class _EpisodeBuffer:
    """Growable pre-allocated (T, C) float buffer for one environment."""

    __slots__ = ("data", "n")

    def __init__(self, capacity: int):
        self.data = np.full((capacity, len(_COLS)), np.nan, dtype=np.float64)
        self.n = 0

    def push(self, row: tuple) -> None:
        if self.n == self.data.shape[0]:  # amortised O(1) growth
            grown = np.full((self.n * 2, len(_COLS)), np.nan, dtype=np.float64)
            grown[: self.n] = self.data
            self.data = grown
        self.data[self.n] = row
        self.n += 1

    def pop_copy(self) -> np.ndarray:
        out = self.data[: self.n].copy()
        self.n = 0
        return out


class TelemetryLogger:
    """
    Parameters
    ----------
    run_name:       file stem -> ``<log_dir>/<run_name>.jsonl`` (one run = one architecture/config)
    n_checkpoints:  number of checkpoints on the track (piste0 = 31, piste1 = 57)
    log_dir:        output directory (created if needed)
    n_envs:         number of independent episode streams handled by this instance
                    (1 inside a VehicleEnv, ``num_envs`` inside an SB3 callback)
    dt:             simulation step in seconds (for lap times)
    meta:           free-form dict copied into every episode record (arch, map, lidar_res, ...)
    capacity:       initial per-env buffer length in steps (grows automatically)
    """

    def __init__(
        self,
        run_name: str,
        n_checkpoints: int,
        log_dir: str | os.PathLike = "logs",
        n_envs: int = 1,
        dt: float = DEFAULT_DT,
        meta: Optional[Dict[str, Any]] = None,
        capacity: int = 4096,
    ):
        self.run_name = run_name
        self.n_checkpoints = int(n_checkpoints)
        self.dt = float(dt)
        self.meta = dict(meta or {})
        self.meta.setdefault("arch", run_name)

        self.path = Path(log_dir) / f"{run_name}.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)

        self._buffers = [_EpisodeBuffer(capacity) for _ in range(n_envs)]
        self._episode_counter = 0
        self._local_ep_counter = [0 for _ in range(n_envs)]
        self._step_counter = 0
        self._start_wall_time = time.time()
        self._lock = threading.Lock()  # only guards counters (end_episode), never log_step

        self._q: "queue.Queue[Optional[dict]]" = queue.Queue(maxsize=10_000)
        self._dropped = 0
        self._writer = threading.Thread(target=self._writer_loop, name=f"telemetry-{run_name}", daemon=True)
        self._writer.start()
        self._closed = False
        atexit.register(self.close)

    # ------------------------------------------------------------------ hot path
    def log_step(
        self,
        speed: float,
        steering: float,
        min_lidar: float,
        checkpoint_id: int,
        reward: float,
        inference_ms: float = _NAN,
        entropy: float = _NAN,
        ts_gap: float = _NAN,
        wm_err: float = _NAN,
        env_id: int = 0,
    ) -> None:
        """Record one simulation step. O(1), no allocation, no I/O."""
        self._buffers[env_id].push(
            (speed, steering, min_lidar, checkpoint_id, reward, inference_ms, entropy, ts_gap, wm_err)
        )

    def end_episode(
        self,
        crashed: bool,
        env_id: int = 0,
        global_step: Optional[int] = None,
        sim_rank: Optional[int] = None,
        vehicle_rank: Optional[int] = None,
        local_ep_idx: Optional[int] = None,
        map_name: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Close the current episode of ``env_id`` and enqueue its summary (non-blocking)."""
        steps = self._buffers[env_id].pop_copy()
        if steps.shape[0] == 0:
            return
        now = time.time()
        with self._lock:
            self._episode_counter += 1
            if 0 <= env_id < len(self._local_ep_counter):
                self._local_ep_counter[env_id] += 1
                default_lep = self._local_ep_counter[env_id]
            else:
                default_lep = self._episode_counter
            self._step_counter += steps.shape[0]
            ep = self._episode_counter
            cum_steps = max(self._step_counter, int(global_step)) if global_step is not None else self._step_counter
            gs = int(global_step) if global_step is not None else cum_steps
            lep = int(local_ep_idx) if local_ep_idx and local_ep_idx > 0 else default_lep
        n_sims = int(self.meta.get("n_sims", 1) or 1)
        job = {
            "kind": "episode",
            "steps": steps,
            "crashed": bool(crashed),
            "env_id": int(env_id),
            "sim_rank": int(sim_rank) if sim_rank is not None else (int(env_id) % n_sims),
            "vehicle_rank": int(vehicle_rank) if vehicle_rank is not None else (int(env_id) // n_sims),
            "local_ep_idx": lep,
            "map_name": map_name or self.meta.get("map", "piste0"),
            "episode": ep,
            "global_step": gs,
            "cum_timesteps": cum_steps,
            "cum_sim_time_s": round(cum_steps * self.dt, 3),
            "wall_time": now,
            "wall_time_elapsed_s": round(now - self._start_wall_time, 3),
            "extra": extra or {},
        }
        self._enqueue(job)

    def log_train(self, global_step: int, **metrics: float) -> None:
        """Log algorithm-level metrics (entropy, approx_kl, value_loss, ...) once per update."""
        now = time.time()
        self._enqueue(
            {
                "kind": "train",
                "global_step": int(global_step),
                "cum_timesteps": int(global_step),
                "cum_sim_time_s": round(int(global_step) * self.dt, 3),
                "wall_time": now,
                "wall_time_elapsed_s": round(now - self._start_wall_time, 3),
                "metrics": metrics,
            }
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._q.put(None)
        self._writer.join(timeout=5.0)

    # ------------------------------------------------------------------ background
    def _enqueue(self, job: dict) -> None:
        try:
            self._q.put_nowait(job)
        except queue.Full:  # never block the simulation; count drops instead
            self._dropped += 1

    def _writer_loop(self) -> None:
        fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
        try:
            while True:
                job = self._q.get()
                if job is None:
                    break
                try:
                    rec = self._summarise(job) if job["kind"] == "episode" else self._train_record(job)
                    line = json.dumps(rec, separators=(",", ":"), allow_nan=False) + "\n"
                    # single write() on an O_APPEND fd -> line-atomic even with several processes
                    os.write(fd, line.encode("utf-8"))
                except Exception as exc:  # telemetry must never crash training
                    print(f"[telemetry] failed to write record: {exc!r}")
        finally:
            os.close(fd)

    def _train_record(self, job: dict) -> dict:
        return {
            "type": "train",
            "v": SCHEMA_VERSION,
            "run_name": self.run_name,
            "arch": self.meta.get("arch"),
            "global_step": job["global_step"],
            "cum_timesteps": job.get("cum_timesteps", job["global_step"]),
            "cum_sim_time_s": job.get("cum_sim_time_s", round(job["global_step"] * self.dt, 3)),
            "wall_time": round(job["wall_time"], 3),
            "wall_time_elapsed_s": job.get("wall_time_elapsed_s"),
            **{k: _f(v, 6) for k, v in job["metrics"].items()},
        }

    def _summarise(self, job: dict) -> dict:
        S = job["steps"]
        T = S.shape[0]
        N = self.n_checkpoints
        col = lambda name: S[:, _IDX[name]]  # noqa: E731

        speed, steer, lidar = col("speed"), col("steering"), col("min_lidar")
        cp = col("checkpoint_id")
        reward, inf_ms = col("reward"), col("inference_ms")

        # --- control smoothness: (1/T) * sum |delta_t - delta_{t-1}|  [rad/step]
        steer_osc = float(np.abs(np.diff(steer)).sum() / T) if T > 1 else 0.0

        # --- checkpoint passes / laps / best lap
        cp_int = np.where(np.isfinite(cp), cp, -1).astype(np.int64)
        known = (cp_int >= 0) & (cp_int < N)
        changes = np.flatnonzero((cp_int[1:] != cp_int[:-1]) & known[1:] & known[:-1]) + 1
        n_pass_cp = int(changes.size)
        n_pass_rew = int(np.sum(np.where(np.isfinite(reward), reward, 0.0) > 0.5))
        n_pass = max(n_pass_cp, n_pass_rew)
        best_lap_s = None
        # CoVAPSy spawns the car ON a checkpoint, so step 0 counts as a crossing.
        crossings = np.concatenate([[0], changes])
        if N > 0 and crossings.size > N:
            lap_steps = crossings[N:] - crossings[:-N]  # N consecutive crossings = 1 full lap
            best_lap_s = _f(lap_steps.min() * self.dt, 3)

        # --- reward density per checkpoint & per second (de-confounded from survival length)
        pos_reward_sum = float(np.nansum(np.where(reward > 0.0, reward, 0.0)))
        survival_seconds = max(self.dt, T * self.dt)
        reward_per_cp = _f(pos_reward_sum / max(1, n_pass), 4)
        reward_per_sec = _f(np.nansum(reward) / survival_seconds, 4)
        track_completion_pct = _f((n_pass / N) * 100.0, 2) if N > 0 else 0.0

        # --- per-checkpoint aggregates (vectorised)
        ids = cp_int[known]
        cnt = np.bincount(ids, minlength=N).astype(np.float64)
        visited = cnt > 0
        with np.errstate(invalid="ignore", divide="ignore"):
            spd_k = np.bincount(ids, weights=speed[known], minlength=N) / cnt
            osc_k = np.zeros(N)
            if T > 1:
                d = np.abs(np.diff(steer, prepend=steer[0]))[known]
                osc_k = np.bincount(ids, weights=d, minlength=N) / cnt
        lid_k = np.full(N, np.inf)
        np.minimum.at(lid_k, ids, np.where(np.isfinite(lidar[known]), lidar[known], np.inf))
        spd_k[~visited] = np.nan
        osc_k[~visited] = np.nan
        lid_k[~np.isfinite(lid_k)] = np.nan

        crash_cp = int(cp_int[-1]) if (job["crashed"] and known[-1]) else None

        # --- latency percentiles
        fin = inf_ms[np.isfinite(inf_ms)]
        if fin.size:
            p50, p95, p99 = np.percentile(fin, [50, 95, 99])
            inference = {
                "mean": _f(fin.mean(), 3), "p50": _f(p50, 3), "p95": _f(p95, 3),
                "p99": _f(p99, 3), "max": _f(fin.max(), 3),
                "over_25ms_pct": _f((fin > 25.0).mean() * 100, 2),
            }
        else:
            inference = None

        # --- reward-hacking detector: Pearson r(step reward, speed) within the episode
        corr_rv = None
        if T > 2 and np.nanstd(reward) > 1e-9 and np.nanstd(speed) > 1e-9:
            corr_rv = _f(np.corrcoef(reward, speed)[0, 1], 4)

        def opt_mean(name):
            a = col(name)
            a = a[np.isfinite(a)]
            return _f(a.mean(), 6) if a.size else None

        fin_lidar = lidar[np.isfinite(lidar)]
        meta_clean = {k: v for k, v in self.meta.items() if k not in ("type", "v", "run_name", "map")}
        return {
            "type": "episode",
            "v": SCHEMA_VERSION,
            "run_name": self.run_name,
            **meta_clean,
            "map": job.get("map_name") or self.meta.get("map", "piste0"),
            "episode": job["episode"],
            "local_ep_idx": job["local_ep_idx"],
            "env_id": job["env_id"],
            "sim_rank": job["sim_rank"],
            "vehicle_rank": job["vehicle_rank"],
            "global_step": job["global_step"],
            "cum_timesteps": job["cum_timesteps"],
            "cum_sim_time_s": job["cum_sim_time_s"],
            "wall_time": round(job["wall_time"], 3),
            "wall_time_elapsed_s": job["wall_time_elapsed_s"],
            "n_checkpoints": N,
            "dt": self.dt,
            "steps": T,
            "duration_s": _f(T * self.dt, 3),
            "total_reward": _f(np.nansum(reward), 4),
            "reward_per_step": _f(np.nanmean(reward), 5),
            "reward_per_cp": reward_per_cp,
            "reward_per_sec": reward_per_sec,
            "mean_speed": _f(np.nanmean(speed), 4),
            "max_speed": _f(np.nanmax(speed), 4),
            "min_lidar": _f(fin_lidar.min(), 4) if fin_lidar.size else None,
            "mean_min_lidar": _f(fin_lidar.mean(), 4) if fin_lidar.size else None,
            "crashed": job["crashed"],
            "crash_checkpoint": crash_cp,
            "start_checkpoint": int(cp_int[0]) if known[0] else None,
            "checkpoints_passed": n_pass,
            "checkpoints_crossed": n_pass,
            "track_completion_pct": track_completion_pct,
            "laps": n_pass // N if N else 0,
            "best_lap_s": best_lap_s,
            "steer_osc": _f(steer_osc, 6),
            "inference_ms": inference,
            "corr_reward_speed": corr_rv,
            "entropy": opt_mean("entropy"),
            "ts_l2_gap": opt_mean("ts_gap"),
            "wm_recon_err": opt_mean("wm_err"),
            "per_checkpoint": {
                "steps": [int(c) for c in cnt],
                "mean_speed": _flist(spd_k, 3),
                "min_lidar": _flist(lid_k, 3),
                "steer_osc": _flist(osc_k, 5),
            },
            **({"extra": job["extra"]} if job["extra"] else {}),
            **({"dropped_records": self._dropped} if self._dropped else {}),
        }
