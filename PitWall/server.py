"""
CoVAPSy Pit-Wall — dashboard server.

Tails every ``<log_dir>/*.jsonl`` incrementally (only new bytes are parsed on
each poll), aggregates the episode summaries with NumPy and serves a
dark-mode F1-style Plotly dashboard.

    uv run python server.py                      # http://127.0.0.1:8050
    uv run python server.py --log-dir ../CoVAPSy/logs --port 8060
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

HERE = Path(__file__).resolve().parent
LIVE_TIMEOUT_S = 10.0  # file touched within this window -> "LIVE"

EPISODE_SCALARS = (
    "global_step", "cum_timesteps", "cum_sim_time_s", "steps", "duration_s",
    "total_reward", "reward_per_step", "reward_per_cp", "reward_per_sec",
    "mean_speed", "max_speed", "min_lidar", "mean_min_lidar",
    "checkpoints_passed", "checkpoints_crossed", "track_completion_pct",
    "laps", "best_lap_s", "steer_osc", "corr_reward_speed", "entropy",
    "ts_l2_gap", "wm_recon_err", "wall_time", "wall_time_elapsed_s",
)
INFERENCE_KEYS = ("p50", "p95", "p99", "max", "mean", "over_25ms_pct")
TRAIN_SKIP = {
    "type", "v", "run_name", "arch", "global_step",
    "cum_timesteps", "cum_sim_time_s", "wall_time", "wall_time_elapsed_s",
}


# ============================================================================ helpers
def jl(a: np.ndarray, nd: int = 4) -> list:
    """NumPy -> JSON list with NaN/inf -> null."""
    a = np.asarray(a, dtype=np.float64)
    out = np.round(a, nd).astype(object)
    out[~np.isfinite(a)] = None
    return out.tolist()


def rolling_mean(x: np.ndarray, w: int) -> np.ndarray:
    """NaN-aware trailing rolling mean (min_periods=1)."""
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return x
    w = max(1, int(w))
    fin = np.isfinite(x)
    c = np.concatenate([[0.0], np.cumsum(np.where(fin, x, 0.0))])
    n = np.concatenate([[0], np.cumsum(fin)])
    idx = np.arange(1, x.size + 1)
    lo = np.maximum(0, idx - w)
    cnt = n[idx] - n[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        r = (c[idx] - c[lo]) / cnt
    r[cnt == 0] = np.nan
    return r


def rolling_std(x: np.ndarray, w: int) -> np.ndarray:
    """NaN-aware trailing rolling standard deviation."""
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return x
    w = max(2, int(w))
    fin = np.isfinite(x)
    x0 = np.where(fin, x, 0.0)
    c1 = np.concatenate([[0.0], np.cumsum(x0)])
    c2 = np.concatenate([[0.0], np.cumsum(x0 * x0)])
    n = np.concatenate([[0], np.cumsum(fin)])
    idx = np.arange(1, x.size + 1)
    lo = np.maximum(0, idx - w)
    cnt = n[idx] - n[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = (c1[idx] - c1[lo]) / cnt
        var = np.maximum(0.0, (c2[idx] - c2[lo]) / cnt - mean * mean)
        std = np.sqrt(var)
    std[cnt < 2] = 0.0
    std[~np.isfinite(x) & (cnt == 0)] = np.nan
    return std


def rolling_corr(x: np.ndarray, y: np.ndarray, w: int, min_periods: int = 5) -> np.ndarray:
    """NaN-aware trailing Pearson correlation r(x, y) over window w."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    n = x.size
    out = np.full(n, np.nan, dtype=np.float64)
    if n == 0:
        return out
    w = max(min_periods, int(w))
    valid = np.isfinite(x) & np.isfinite(y)
    for i in range(n):
        lo = max(0, i + 1 - w)
        m = valid[lo : i + 1]
        if m.sum() < min_periods:
            continue
        xv = x[lo : i + 1][m]
        yv = y[lo : i + 1][m]
        sx = xv.std()
        sy = yv.std()
        if sx > 1e-9 and sy > 1e-9:
            out[i] = float(np.corrcoef(xv, yv)[0, 1])
    return out


def downsample_idx(n: int, max_points: int) -> np.ndarray:
    if n <= max_points:
        return np.arange(n)
    return np.unique(np.linspace(0, n - 1, max_points).round().astype(int))


# ============================================================================ run cache
class RunCache:
    """Incrementally-parsed contents of one ``<run>.jsonl`` file."""

    def __init__(self, path: Path):
        self.path = path
        self.name = path.stem
        self._reset()

    def _reset(self) -> None:
        self.offset = 0
        self.episodes: List[dict] = []
        self.train: List[dict] = []
        self.meta: Dict[str, Any] = {}
        self.mtime = 0.0
        self.bad_lines = 0
        self._cols_cache: Optional[tuple] = None

    def refresh(self) -> None:
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return
        if st.st_size < self.offset:  # truncated / rewritten (e.g. mock --reset)
            self._reset()
        self.mtime = st.st_mtime
        if st.st_size == self.offset:
            return
        with open(self.path, "rb") as f:
            f.seek(self.offset)
            chunk = f.read(st.st_size - self.offset)
        end = chunk.rfind(b"\n")
        if end < 0:  # partial line only, wait for the writer to finish it
            return
        self.offset += end + 1
        for raw in chunk[: end + 1].splitlines():
            if not raw.strip():
                continue
            try:
                rec = json.loads(raw)
            except json.JSONDecodeError:
                self.bad_lines += 1
                continue
            kind = rec.get("type", "episode")
            if kind == "episode":
                self.episodes.append(rec)
                for k in ("arch", "algo", "map", "n_checkpoints", "lidar_res", "n_envs", "n_sims", "n_vehicles", "notes"):
                    if k in rec and rec.get(k) is not None:
                        self.meta[k] = rec.get(k)
            elif kind == "train":
                self.train.append(rec)
        self._cols_cache = None

    @property
    def live(self) -> bool:
        return (time.time() - self.mtime) < LIVE_TIMEOUT_S

    def distinct_maps(self) -> List[str]:
        maps = sorted({str(e.get("map") or self.meta.get("map") or "piste0") for e in self.episodes})
        return maps if maps else [str(self.meta.get("map") or "piste0")]

    # ---------------------------------------------------------------- columns
    def columns(self, map_filter: Optional[str] = None) -> tuple[List[dict], dict]:
        """Columnar NumPy view of episodes (filtered by map if map_filter is provided)."""
        if map_filter is None and self._cols_cache is not None and self._cols_cache[0] == len(self.episodes):
            return self.episodes, self._cols_cache[1]

        if map_filter is not None:
            eps = [
                e for e in self.episodes
                if str(e.get("map") or self.meta.get("map") or "piste0") == map_filter
            ]
        else:
            eps = self.episodes

        nan = np.nan
        cols: Dict[str, Any] = {
            k: np.array([e.get(k) if e.get(k) is not None else nan for e in eps], dtype=np.float64)
            for k in EPISODE_SCALARS
        }
        cols["crashed"] = np.array([bool(e.get("crashed")) for e in eps], dtype=bool)
        cols["crash_checkpoint"] = np.array(
            [e["crash_checkpoint"] if e.get("crash_checkpoint") is not None else -1 for e in eps], dtype=np.int64
        )
        cols["env_id"] = np.array([int(e.get("env_id", 0) or 0) for e in eps], dtype=np.int64)
        cols["sim_rank"] = np.array(
            [int(e["sim_rank"]) if e.get("sim_rank") is not None else -1 for e in eps], dtype=np.int64
        )
        cols["vehicle_rank"] = np.array(
            [int(e["vehicle_rank"]) if e.get("vehicle_rank") is not None else -1 for e in eps], dtype=np.int64
        )

        # Reconstruct local_ep_idx per env_id if missing in older logs
        local_ep = np.zeros(len(eps), dtype=np.int64)
        env_counts: Dict[int, int] = {}
        for idx_e, e in enumerate(eps):
            eid = int(e.get("env_id", 0) or 0)
            env_counts[eid] = env_counts.get(eid, 0) + 1
            lep = e.get("local_ep_idx")
            local_ep[idx_e] = int(lep) if (lep is not None and int(lep) > 0) else env_counts[eid]
        cols["local_ep_idx"] = local_ep

        for k in INFERENCE_KEYS:
            vals = [(e.get("inference_ms") or {}).get(k) for e in eps]
            cols[f"inf_{k}"] = np.array([v if v is not None else nan for v in vals], dtype=np.float64)

        cols["n_checkpoints"] = np.array([int(e.get("n_checkpoints", 0) or 31) for e in eps], dtype=np.int64)
        cols["dt"] = np.array([float(e.get("dt", 0.025) or 0.025) for e in eps], dtype=np.float64)
        cols["map"] = [str(e.get("map") or self.meta.get("map") or "piste0") for e in eps]

        # Backfill cumulative timesteps & cumulative sim time for legacy logs
        steps_clean = np.nan_to_num(cols["steps"], nan=0.0)
        cum_steps_fallback = np.cumsum(steps_clean)
        gs = cols["global_step"]
        if np.isnan(gs).all():
            gs = cum_steps_fallback
        else:
            gs = np.where(np.isfinite(gs), gs, cum_steps_fallback)
        gs = np.maximum.accumulate(gs)
        cols["global_step"] = gs

        ct = cols["cum_timesteps"]
        ct = np.where(np.isfinite(ct), ct, gs)
        ct = np.maximum.accumulate(ct)
        cols["cum_timesteps"] = ct

        cs = cols["cum_sim_time_s"]
        cs_fallback = ct * cols["dt"]
        cs = np.where(np.isfinite(cs), cs, cs_fallback)
        cs = np.maximum.accumulate(cs)
        cols["cum_sim_time_s"] = cs

        # Backfill checkpoints_crossed, track_completion_pct, reward_per_cp, reward_per_sec
        cp_crossed = np.where(
            np.isfinite(cols["checkpoints_crossed"]),
            cols["checkpoints_crossed"],
            np.where(np.isfinite(cols["checkpoints_passed"]), cols["checkpoints_passed"], 0.0),
        )
        cols["checkpoints_crossed"] = cp_crossed

        N_arr = np.maximum(1, cols["n_checkpoints"]).astype(np.float64)
        tcp = np.where(
            np.isfinite(cols["track_completion_pct"]),
            cols["track_completion_pct"],
            (cp_crossed / N_arr) * 100.0,
        )
        cols["track_completion_pct"] = tcp

        dur_s = np.maximum(cols["dt"], steps_clean * cols["dt"])
        rps_sec = np.where(
            np.isfinite(cols["reward_per_sec"]),
            cols["reward_per_sec"],
            cols["total_reward"] / dur_s,
        )
        cols["reward_per_sec"] = rps_sec

        # Positive reward estimate for legacy logs: total_reward + 0.5 * crashed
        pos_rew_est = np.maximum(0.0, cols["total_reward"] + np.where(cols["crashed"], 0.5, 0.0))
        rp_cp = np.where(
            np.isfinite(cols["reward_per_cp"]),
            cols["reward_per_cp"],
            pos_rew_est / np.maximum(1.0, cp_crossed),
        )
        cols["reward_per_cp"] = rp_cp

        # Effective sector reward density: reward_per_cp when checkpoints_crossed > 0 else reward_per_sec
        cols["reward_density"] = np.where(cp_crossed > 0, rp_cp, rps_sec)

        if map_filter is None:
            self._cols_cache = (len(eps), cols)
        return eps, cols

    # ---------------------------------------------------------------- cohort aggregation
    def _cohort_series(self, c: dict) -> dict:
        """
        Group episodes across parallel vehicles/sims on the same map by local_ep_idx
        (Cohort Round k) so fast-crashing envs never flood the rolling window.
        Returns cohort-averaged arrays and inter-vehicle std arrays.
        """
        n = len(c["steps"])
        unique_envs = np.unique(c["env_id"])
        n_envs = max(int(self.meta.get("n_envs", 1) or 1), int(unique_envs.size))
        metric_keys = (
            "mean_speed", "steps", "crashed_pct", "steer_osc",
            "inf_p50", "inf_p95", "inf_max", "reward_per_step",
            "reward_per_cp", "reward_per_sec", "reward_density",
            "total_reward", "corr_reward_speed", "entropy",
            "ts_l2_gap", "wm_recon_err", "min_lidar",
            "checkpoints_crossed", "track_completion_pct",
        )
        raw = {k: c[k] for k in metric_keys if k != "crashed_pct"}
        raw["crashed_pct"] = c["crashed"].astype(np.float64) * 100.0

        if n_envs <= 1 or unique_envs.size <= 1:
            out = {
                "cohort_round": np.arange(1, n + 1, dtype=np.float64),
                "cum_timesteps": c["cum_timesteps"],
                "cum_sim_time_s": c["cum_sim_time_s"],
                "n_envs": n_envs,
                "is_cohort": False,
            }
            for k, arr in raw.items():
                out[k] = arr
                out[f"{k}_std"] = np.zeros_like(arr)
            return out

        lep = c["local_ep_idx"]
        rounds = np.unique(lep)
        m = rounds.size
        out = {
            "cohort_round": rounds.astype(np.float64),
            "cum_timesteps": np.zeros(m, dtype=np.float64),
            "cum_sim_time_s": np.zeros(m, dtype=np.float64),
            "n_envs": n_envs,
            "is_cohort": True,
        }
        for k in raw:
            out[k] = np.full(m, np.nan, dtype=np.float64)
            out[f"{k}_std"] = np.zeros(m, dtype=np.float64)

        for r_idx, r_val in enumerate(rounds):
            mask = lep == r_val
            out["cum_timesteps"][r_idx] = float(np.nanmax(c["cum_timesteps"][mask]))
            out["cum_sim_time_s"][r_idx] = float(np.nanmax(c["cum_sim_time_s"][mask]))
            for k, arr in raw.items():
                vals = arr[mask]
                fin = vals[np.isfinite(vals)]
                if fin.size > 0:
                    out[k][r_idx] = float(fin.mean())
                    out[f"{k}_std"][r_idx] = float(fin.std()) if fin.size > 1 else 0.0

        out["cum_timesteps"] = np.maximum.accumulate(out["cum_timesteps"])
        out["cum_sim_time_s"] = np.maximum.accumulate(out["cum_sim_time_s"])
        return out

    # ---------------------------------------------------------------- aggregation
    def summary(
        self,
        x_mode: str,
        window: int,
        cp_last: int,
        max_points: int,
        map_filter: Optional[str] = None,
        display_name: Optional[str] = None,
    ) -> dict:
        eps, c = self.columns(map_filter=map_filter)
        n = len(eps)
        rname = display_name or self.name
        cur_map = map_filter or (c["map"][-1] if n > 0 else str(self.meta.get("map") or "piste0"))

        unique_envs = np.unique(c["env_id"]) if n > 0 else np.array([0])
        n_envs = max(int(self.meta.get("n_envs", 1) or 1), int(unique_envs.size))
        valid_sims = c["sim_rank"][c["sim_rank"] >= 0] if n > 0 else np.array([])
        n_sims = int(self.meta.get("n_sims") or ( np.unique(valid_sims).size if valid_sims.size else 1 ))
        valid_vehs = c["vehicle_rank"][c["vehicle_rank"] >= 0] if n > 0 else np.array([])
        n_vehicles = int(
            self.meta.get("n_vehicles")
            or (np.unique(valid_vehs).size if valid_vehs.size else max(1, n_envs // max(1, n_sims)))
        )

        meta_out = {
            **self.meta,
            "map": cur_map,
            "n_envs": n_envs,
            "n_sims": n_sims,
            "n_vehicles": n_vehicles,
        }
        base = {
            "run_name": rname,
            "base_run_name": self.name,
            "arch": self.meta.get("arch", self.name),
            "meta": meta_out,
            "live": self.live,
            "last_update": self.mtime,
            "episodes": n,
            "bad_lines": self.bad_lines,
        }
        if n == 0:
            return {**base, "empty": True}

        coh = self._cohort_series(c)
        m = coh["cohort_round"].size
        dt_last = float(c["dt"][-1]) if n > 0 else 0.025

        # X-axis selection: "sim_time" (min), "step" (k timesteps), or "episode" (cohort round)
        if x_mode == "sim_time":
            x_full = coh["cum_sim_time_s"] / 60.0
        elif x_mode == "step":
            x_full = coh["cum_timesteps"] / 1000.0
        else:
            x_full = coh["cohort_round"]

        # Rolling Pearson r between mean_speed and reward_density (reward_per_cp)
        corr_window = max(10, int(window))
        rolling_r = rolling_corr(coh["mean_speed"], coh["reward_density"], corr_window, min_periods=4)
        # Fallback to within-episode corr if < 4 points at very start of training
        corr_combined = np.where(np.isfinite(rolling_r), rolling_r, coh["corr_reward_speed"])

        series_src = {
            "mean_speed": coh["mean_speed"],
            "survival": coh["steps"],
            "crash_rate_roll": coh["crashed_pct"],
            "steer_osc": coh["steer_osc"],
            "inf_p50": coh["inf_p50"],
            "inf_p95": coh["inf_p95"],
            "inf_max": coh["inf_max"],
            "reward_per_step": coh["reward_per_step"],
            "reward_per_cp": coh["reward_per_cp"],
            "reward_per_sec": coh["reward_per_sec"],
            "reward_density": coh["reward_density"],
            "total_reward": coh["total_reward"],
            "corr_rv": corr_combined,
            "entropy_ep": coh["entropy"],
            "ts_gap": coh["ts_l2_gap"],
            "wm_err": coh["wm_recon_err"],
            "min_lidar": coh["min_lidar"],
            "checkpoints_crossed": coh["checkpoints_crossed"],
            "track_completion_pct": coh["track_completion_pct"],
        }
        rolled = {k: rolling_mean(v, window) for k, v in series_src.items()}
        rolled["crash_rate_cum"] = np.cumsum(coh["crashed_pct"]) / np.arange(1, m + 1, dtype=np.float64)

        # Confidence bands (+/- 1 sigma combining cohort inter-vehicle std and rolling std)
        for band_key, src_key in (
            ("mean_speed", "mean_speed"),
            ("survival", "steps"),
            ("reward_density", "reward_density"),
            ("corr_rv", "corr_reward_speed"),
            ("entropy_ep", "entropy"),
        ):
            coh_std = rolling_mean(coh.get(f"{src_key}_std", np.zeros(m)), window)
            win_std = rolling_std(series_src[band_key], max(3, window))
            tot_std = np.sqrt(np.nan_to_num(coh_std) ** 2 + np.nan_to_num(win_std) ** 2)
            rolled[f"{band_key}_std"] = tot_std

        idx = downsample_idx(m, max_points)
        series = {
            "x": jl(x_full[idx], 3),
            "cohort_round": jl(coh["cohort_round"][idx], 0),
            "cum_sim_time_min": jl((coh["cum_sim_time_s"] / 60.0)[idx], 3),
            "cum_timesteps_k": jl((coh["cum_timesteps"] / 1000.0)[idx], 2),
        }
        series.update({k: jl(v[idx], 5) for k, v in rolled.items()})
        series["inf_p95_raw"] = jl(coh["inf_p95"][idx], 3)
        if np.isfinite(rolled["entropy_ep"]).any():
            series["k_eff_ep"] = jl(np.exp(rolled["entropy_ep"][idx] / 2.0), 3)
        else:
            series["k_eff_ep"] = [None] * len(idx)

        # ---------------- recent-window (current model state) slice
        lo = 0 if cp_last <= 0 else max(0, n - cp_last)
        recent = slice(lo, n)
        ep_idx_raw = np.arange(1, n + 1, dtype=np.float64)
        scatter = {
            "mean_speed": jl(c["mean_speed"][recent], 4),
            "steer_osc": jl(c["steer_osc"][recent], 6),
            "reward_per_step": jl(c["reward_per_step"][recent], 5),
            "reward_per_cp": jl(c["reward_per_cp"][recent], 4),
            "reward_density": jl(c["reward_density"][recent], 4),
            "episode": jl(ep_idx_raw[recent], 0),
        }

        checkpoint = self._checkpoint_agg(eps, c, lo, n)

        # ---------------- train records (entropy, KL, K_eff, ...)
        train = None
        if self.train:
            tg = np.array([t.get("global_step", np.nan) for t in self.train], dtype=np.float64)
            if x_mode == "episode":
                tx = np.searchsorted(coh["cum_timesteps"], tg, side="left") + 1.0
            elif x_mode == "sim_time":
                tsim = np.array(
                    [t.get("cum_sim_time_s", (t.get("global_step") or 0) * dt_last) for t in self.train],
                    dtype=np.float64,
                )
                tx = tsim / 60.0
            else:
                tx = tg / 1000.0
            keys = sorted(
                {k for t in self.train for k, v in t.items() if k not in TRAIN_SKIP and isinstance(v, (int, float))}
            )
            tidx = downsample_idx(len(self.train), max_points)
            train = {"x": jl(tx[tidx], 3)}
            for k in keys:
                arr = np.array([t.get(k) if t.get(k) is not None else np.nan for t in self.train], dtype=np.float64)
                train[k] = jl(arr[tidx], 6)
                if k == "entropy":
                    train["k_eff"] = jl(np.exp(arr[tidx] / 2.0), 3)

        # ---------------- KPI tiles (rolling = "current form")
        def last(a):
            fin = a[np.isfinite(a)]
            return float(fin[-1]) if fin.size else None

        best_lap = c["best_lap_s"][np.isfinite(c["best_lap_s"])]
        N_cp = int(c["n_checkpoints"][-1]) if n > 0 else 31
        ent_last = (train or {}).get("entropy", [None])[-1] if train else last(rolled["entropy_ep"])
        k_eff_last = float(np.exp(ent_last / 2.0)) if (ent_last is not None and np.isfinite(ent_last)) else None
        max_cp_reached = int(np.nanmax(c["checkpoints_crossed"])) if n > 0 else 0

        kpi = {
            "mean_speed": last(rolled["mean_speed"]),
            "crash_rate": last(rolled["crash_rate_roll"]),
            "survival_s": (last(rolled["survival"]) or 0) * dt_last,
            "steer_osc": last(rolled["steer_osc"]),
            "p95_ms": last(rolled["inf_p95"]),
            "best_lap_s": float(best_lap.min()) if best_lap.size else None,
            "laps": int(np.nansum(c["laps"])),
            "mean_cp_pct": last(rolled["track_completion_pct"]),
            "mean_cp": last(rolled["checkpoints_crossed"]),
            "max_cp": max_cp_reached,
            "n_checkpoints": N_cp,
            "total_steps": float(coh["cum_timesteps"][-1]),
            "cum_sim_time_min": float(coh["cum_sim_time_s"][-1] / 60.0),
            "entropy": ent_last,
            "k_eff": k_eff_last,
            "corr_rv": last(rolled["corr_rv"]),
            "map": cur_map,
            "n_sims": n_sims,
            "n_vehicles": n_vehicles,
            "n_envs": n_envs,
            "cohort_rounds": int(m),
        }

        return {**base, "kpi": kpi, "series": series, "scatter": scatter, "checkpoint": checkpoint, "train": train}

    def _checkpoint_agg(self, eps: List[dict], c: dict, lo: int, n: int) -> Optional[dict]:
        # Only aggregate episodes from the same track as the latest episode in this slice.
        N = int(c["n_checkpoints"][-1])
        if N <= 0:
            return None
        cur_map = c["map"][-1]
        sel = [
            i for i in range(lo, n)
            if c["n_checkpoints"][i] == N and c["map"][i] == cur_map
            and (eps[i].get("per_checkpoint") or {}).get("steps")
        ]
        if not sel:
            return None

        steps = np.zeros((len(sel), N))
        spd = np.full((len(sel), N), np.nan)
        lid = np.full((len(sel), N), np.nan)
        osc = np.full((len(sel), N), np.nan)
        for r, i in enumerate(sel):
            pc = eps[i]["per_checkpoint"]
            steps[r] = np.asarray(pc["steps"], dtype=np.float64)[:N]
            spd[r] = np.array([v if v is not None else np.nan for v in pc["mean_speed"][:N]])
            lid[r] = np.array([v if v is not None else np.nan for v in pc["min_lidar"][:N]])
            if pc.get("steer_osc"):
                osc[r] = np.array([v if v is not None else np.nan for v in pc["steer_osc"][:N]])

        tot_steps = steps.sum(axis=0)
        with np.errstate(invalid="ignore", divide="ignore"):
            mean_speed = np.nansum(spd * steps, axis=0) / tot_steps          # step-weighted
            mean_osc = np.nansum(osc * steps, axis=0) / tot_steps
        mean_speed[tot_steps == 0] = np.nan
        mean_osc[tot_steps == 0] = np.nan
        with np.errstate(all="ignore"):
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", category=RuntimeWarning)
                lid_mean = np.nanmean(lid, axis=0)                           # typical closest approach
                lid_abs = np.nanmin(lid, axis=0)                             # worst case

        visits = (steps > 0).sum(axis=0).astype(np.float64)
        crash_cps = c["crash_checkpoint"][sel]
        crashed = c["crashed"][sel]
        crash_cps = crash_cps[crashed & (crash_cps >= 0) & (crash_cps < N)]
        crash_count = np.bincount(crash_cps, minlength=N).astype(np.float64)
        total_crashes = crash_count.sum()
        with np.errstate(invalid="ignore", divide="ignore"):
            crash_pct = crash_count / total_crashes * 100.0 if total_crashes else np.zeros(N)
            crash_per_visit = np.where(visits > 0, crash_count / visits * 100.0, np.nan)

        return {
            "n_checkpoints": N,
            "map": cur_map,
            "n_episodes": len(sel),
            "ids": list(range(N)),
            "mean_speed": jl(mean_speed, 4),
            "min_lidar_mean": jl(lid_mean, 4),
            "min_lidar_abs": jl(lid_abs, 4),
            "steer_osc": jl(mean_osc, 6),
            "crash_count": jl(crash_count, 0),
            "crash_pct": jl(crash_pct, 3),
            "crash_per_visit": jl(crash_per_visit, 3),
            "visits": jl(visits, 0),
            "total_crashes": int(total_crashes),
        }


# ============================================================================ registry
class Registry:
    def __init__(self, log_dir: Path):
        self.log_dir = log_dir
        self.runs: Dict[str, RunCache] = {}
        self.lock = threading.Lock()

    def scan(self) -> None:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        present = {p.stem: p for p in self.log_dir.glob("*.jsonl")}
        with self.lock:
            for name, p in present.items():
                if name not in self.runs:
                    self.runs[name] = RunCache(p)
            for name in list(self.runs):
                if name not in present:
                    del self.runs[name]
            for rc in self.runs.values():
                rc.refresh()


def create_app(log_dir: Path) -> FastAPI:
    reg = Registry(log_dir)
    app = FastAPI(title="CoVAPSy Pit-Wall", docs_url="/api/docs")
    static_dir = HERE / "static"
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    @app.get("/")
    def index():
        return FileResponse(static_dir / "index.html")

    @app.get("/api/runs")
    def list_runs():
        reg.scan()
        out = []
        with reg.lock:
            for rc in reg.runs.values():
                maps = rc.distinct_maps()
                if len(maps) <= 1:
                    out.append(
                        {
                            "run_name": rc.name,
                            "arch": rc.meta.get("arch", rc.name),
                            "map": maps[0],
                            "episodes": len(rc.episodes),
                            "live": rc.live,
                            "last_update": rc.mtime,
                            "size_kb": round(rc.offset / 1024, 1),
                        }
                    )
                else:
                    # Automatically split multi-map log files into distinct virtual series
                    for m in maps:
                        ep_count = sum(
                            1 for e in rc.episodes
                            if str(e.get("map") or rc.meta.get("map") or "piste0") == m
                        )
                        out.append(
                            {
                                "run_name": f"{rc.name} [{m}]",
                                "arch": rc.meta.get("arch", rc.name),
                                "map": m,
                                "episodes": ep_count,
                                "live": rc.live,
                                "last_update": rc.mtime,
                                "size_kb": round(rc.offset / 1024, 1),
                            }
                        )
        out.sort(key=lambda r: (not r["live"], -r["last_update"], r["run_name"]))
        return {"log_dir": str(reg.log_dir), "runs": out, "server_time": time.time()}

    @app.get("/api/data")
    def data(
        runs: str = Query("", description="comma-separated run names"),
        x: str = Query("sim_time", pattern="^(sim_time|step|episode)$"),
        window: int = Query(50, ge=1, le=5000),
        cp_last: int = Query(300, ge=0, description="episodes used for checkpoint panels, 0 = all"),
        max_points: int = Query(1500, ge=50, le=20000),
    ):
        reg.scan()
        names = [r for r in runs.split(",") if r]
        out = {}
        with reg.lock:
            for name in names:
                map_filter = None
                base_name = name
                if name.endswith("]") and " [" in name:
                    base_name, map_part = name.rsplit(" [", 1)
                    map_filter = map_part[:-1]
                rc = reg.runs.get(base_name)  # whitelist: only discovered files, no path traversal
                if rc is None:
                    continue
                maps = rc.distinct_maps()
                if map_filter is None and len(maps) > 1:
                    for m in maps:
                        vname = f"{rc.name} [{m}]"
                        out[vname] = rc.summary(x, window, cp_last, max_points, map_filter=m, display_name=vname)
                else:
                    out[name] = rc.summary(x, window, cp_last, max_points, map_filter=map_filter, display_name=name)
        if names and not out:
            raise HTTPException(404, "none of the requested runs exist")
        return JSONResponse({"runs": out, "server_time": time.time()})

    return app


def main() -> None:
    ap = argparse.ArgumentParser(description="CoVAPSy Pit-Wall dashboard")
    ap.add_argument("--log-dir", default=str(HERE / "logs"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8050)
    args = ap.parse_args()
    app = create_app(Path(args.log_dir).resolve())
    print(f"[pit-wall] watching {Path(args.log_dir).resolve()}  ->  http://{args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
