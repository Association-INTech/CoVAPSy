"""
CoVAPSy Pit-Wall — mock live-run generator.

Simulates headless training of several architectures on the real piste0
checkpoint geometry and streams them through the *real* TelemetryLogger,
so the dashboard can be tested end-to-end without Webots.

    uv run python generate_mock_runs.py                 # 3 runs, 150 backfilled episodes, then live
    uv run python generate_mock_runs.py --extras        # + TeacherStudent_Distill & Dreamer_V3 slots
    uv run python generate_mock_runs.py --reset --rate 3

Behaviour baked into the mock (so every panel has something to show):
  * PPO_Legacy_1080   : slow learner, jittery steering, ~34 ms inference (often > 25 ms budget),
                        and late "reward hacking" (learns to crawl to farm the +0.05 survival reward)
  * PPO_NoDropout_Norm: fast learner, fast inference (~11 ms), moderate jitter
  * MemoryCNN_K5      : smooth steering (temporal context), fewer hairpin crashes, ~17 ms
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from telemetry_logger import TelemetryLogger

HERE = Path(__file__).resolve().parent
DT = 0.025
V_MIN, V_MAX = 1.0 / 3.6, 9.0 / 3.6  # CoVAPSy speed range in m/s
MAX_STEPS = 1200                      # 30 s episode cap (truncation)

# piste0.wbt checkpoints (x, y) from checkpoint_manager.py
PISTE0 = np.array([
    (-0.314494, -2.47211), (1.11162, -2.56708), (2.54552, -2.27446), (3.58779, -1.38814),
    (3.58016, -0.0800134), (3.23981, 1.26309), (2.8261, 1.99783), (3.18851, 2.71151),
    (3.6475, 4.09688), (3.1775, 4.44688), (2.58692, 4.5394), (1.52457, 4.3991),
    (0.659969, 3.57074), (0.000799585, 2.90417), (0.0727115, 1.81299), (0.788956, 1.22248),
    (1.24749, 0.288391), (0.88749, -0.281609), (0.0789172, -0.557653), (-0.832859, -0.484867),
    (-1.79723, 0.408769), (-1.7446, 1.3386), (-1.92104, 2.72452), (-2.96264, 2.96666),
    (-4.19027, 2.74619), (-4.34725, 1.7503), (-4.26858, 0.259482), (-4.20936, -1.06968),
    (-4.0021, -2.35518), (-2.89371, -2.49154), (-2.01029, -2.51669),
])
N_CP = len(PISTE0)


def track_profile():
    """Sector length (m) and corner severity in [0, 1] for 'next checkpoint = k'."""
    prev = np.roll(PISTE0, 1, axis=0)
    nxt = np.roll(PISTE0, -1, axis=0)
    seg_in = PISTE0 - prev
    seg_out = nxt - PISTE0
    length = np.linalg.norm(seg_in, axis=1)
    h_in = np.arctan2(seg_in[:, 1], seg_in[:, 0])
    h_out = np.arctan2(seg_out[:, 1], seg_out[:, 0])
    turn = (h_out - h_in + np.pi) % (2 * np.pi) - np.pi      # signed heading change at checkpoint k
    sev = np.clip(np.abs(turn) / (np.pi / 2), 0, 1)
    sev = 0.6 * sev + 0.2 * np.roll(sev, 1) + 0.2 * np.roll(sev, -1)  # corners spill over sectors
    return length, sev / sev.max(), np.sign(turn)


SECTOR_LEN, SEVERITY, TURN_SIGN = track_profile()


@dataclass
class Arch:
    name: str
    algo: str
    lidar_res: int
    learn_tau: float      # episodes to reach ~63 % of the skill cap
    skill_cap: float
    jitter: float         # steering noise std (rad)
    smooth: float         # AR(1) coefficient of steering noise (temporal memory => smoother)
    inf_mu: float         # median inference ms
    inf_sigma: float      # lognormal sigma
    spike_p: float        # probability of a GC/thermal latency spike
    crash_base: float
    hack: bool = False
    ts_gap: bool = False
    wm_err: bool = False
    ent_final: float = 3.0


ARCHS = [
    Arch("PPO_Legacy_1080", "PPO", 1080, 260, 0.72, 0.11, 0.05, 34.0, 0.22, 0.020, 0.0022, hack=True, ent_final=3.9),
    Arch("PPO_NoDropout_Norm", "PPO", 1024, 120, 0.88, 0.068, 0.35, 11.0, 0.18, 0.004, 0.0011, ent_final=3.1),
    Arch("MemoryCNN_K5", "PPO", 128, 150, 0.96, 0.032, 0.82, 17.0, 0.15, 0.003, 0.00065, ent_final=2.7),
]
EXTRAS = [
    Arch("TeacherStudent_Distill", "PPO+BC", 128, 100, 0.92, 0.045, 0.65, 4.5, 0.20, 0.002, 0.0008, ts_gap=True, ent_final=2.9),
    Arch("Dreamer_V3", "DreamerV3", 128, 180, 0.94, 0.040, 0.72, 22.0, 0.20, 0.006, 0.00075, wm_err=True, ent_final=2.5),
]


class MockRun:
    def __init__(self, arch: Arch, log_dir: Path, seed: int):
        self.a = arch
        self.rng = np.random.default_rng(seed)
        self.tl = TelemetryLogger(
            arch.name, N_CP, log_dir, dt=DT,
            meta={"arch": arch.name, "algo": arch.algo, "map": "piste0", "lidar_res": arch.lidar_res, "mock": True},
        )
        self.episode = 0
        self.global_step = 0
        self._next_update = 4096

    def skill(self) -> float:
        a = self.a
        return a.skill_cap * (1 - math.exp(-self.episode / a.learn_tau)) + 0.05

    def speed_pref(self) -> float:
        if not self.a.hack:
            return 1.0
        # late-training collapse: policy discovers that crawling farms survival reward
        return 1.0 - 0.42 / (1 + math.exp(-(self.episode - 180) / 30))

    def run_episode(self) -> None:
        a, rng = self.a, self.rng
        sk, pref = self.skill(), self.speed_pref()
        prog = min(1.0, self.episode / (3 * a.learn_tau))

        nrm = rng.normal(size=(MAX_STEPS, 4))
        unif = rng.random(size=(MAX_STEPS, 2))
        inf = rng.lognormal(math.log(a.inf_mu), a.inf_sigma, size=MAX_STEPS)
        spikes = unif[:, 1] < a.spike_p
        inf[spikes] *= rng.uniform(2.5, 5.0, size=spikes.sum())

        k = (int(rng.integers(0, N_CP - 1)) + 1) % N_CP  # spawn on a checkpoint, head to the next
        s, v, steer, noise = 0.0, V_MIN, 0.0, 0.0
        crashed = False
        cautious = 0.8 if a.hack else 1.0

        for t in range(MAX_STEPS):
            sev = SEVERITY[k]
            v_tgt = V_MIN + (V_MAX - V_MIN) * sk * pref * (1 - 0.55 * sev) + 0.05 * nrm[t, 0]
            v += float(np.clip(v_tgt - v, -3.0 * DT, 2.0 * DT))  # accel / brake limits
            v = max(V_MIN, min(V_MAX, v))

            noise = a.smooth * noise + (1 - a.smooth) * a.jitter * nrm[t, 1] / max(0.35, math.sqrt(1 - a.smooth))
            base = TURN_SIGN[k] * sev * 0.32 * math.sin(math.pi * min(1.0, s / SECTOR_LEN[k]))
            steer = max(-0.4, min(0.4, base + noise))

            vn = v / V_MAX
            clearance = 0.46 - 0.28 * sev * vn ** 1.4 * (1.20 - sk) - 0.55 * abs(noise) + 0.02 * nrm[t, 2]
            clearance = max(0.03, clearance * (1.15 if a.hack else 1.0))

            p_crash = a.crash_base * cautious * (1 + 18 * sev * vn ** 2) * max(0.08, (1.02 - sk) ** 1.6)
            p_crash += 0.025 if clearance < 0.065 else 0.0

            reward = 0.05
            s += v * DT
            if s >= SECTOR_LEN[k]:
                s -= SECTOR_LEN[k]
                k = (k + 1) % N_CP
                reward = 1.0
            if unif[t, 0] < p_crash:
                reward, crashed = -0.5, True

            ts = (0.9 * math.exp(-self.episode / 150) + 0.05 + 0.02 * nrm[t, 3]) if a.ts_gap else float("nan")
            wm = (0.35 * math.exp(-self.episode / 220) + 0.02 + 0.005 * abs(nrm[t, 3])) if a.wm_err else float("nan")

            self.tl.log_step(speed=v, steering=steer, min_lidar=clearance, checkpoint_id=k,
                             reward=reward, inference_ms=float(inf[t]), ts_gap=ts, wm_err=wm)
            if crashed:
                break

        steps = t + 1
        self.global_step += steps
        self.episode += 1
        self.tl.end_episode(crashed=crashed, global_step=self.global_step)

        while self.global_step >= self._next_update:  # one PPO update per 4096 steps
            self._log_update(prog)
            self._next_update += 4096

    def _log_update(self, prog: float) -> None:
        a, rng = self.a, self.rng
        ent = 5.545 - (5.545 - a.ent_final) * prog + 0.05 * rng.normal()  # 2 * ln(16) at start
        self.tl.log_train(
            self.global_step,
            entropy=ent, entropy_loss=-ent,
            approx_kl=abs(0.012 + 0.006 * rng.normal()) * (1.6 if a.name.startswith("PPO_Legacy") else 1.0),
            clip_fraction=max(0.0, 0.14 - 0.06 * prog + 0.02 * rng.normal()),
            explained_variance=min(0.98, 0.05 + 0.85 * prog + 0.04 * rng.normal()),
            value_loss=abs(0.6 * (1 - prog) + 0.08 + 0.05 * rng.normal()),
            policy_gradient_loss=-abs(0.02 * (1 - 0.5 * prog) + 0.004 * rng.normal()),
            fps=abs(140 - a.inf_mu * 1.8 + 6 * rng.normal()),
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="Stream mock CoVAPSy telemetry into logs/*.jsonl")
    ap.add_argument("--log-dir", default=str(HERE / "logs"))
    ap.add_argument("--backfill", type=int, default=150, help="episodes written instantly per run")
    ap.add_argument("--rate", type=float, default=1.5, help="live episodes per second per run")
    ap.add_argument("--episodes", type=int, default=0, help="stop after N total episodes per run (0 = forever)")
    ap.add_argument("--extras", action="store_true", help="add TeacherStudent_Distill and Dreamer_V3 runs")
    ap.add_argument("--reset", action="store_true", help="delete existing mock run files first")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    archs = ARCHS + (EXTRAS if args.extras else [])
    if args.reset:
        for a in archs:
            (log_dir / f"{a.name}.jsonl").unlink(missing_ok=True)

    runs = [MockRun(a, log_dir, args.seed + i) for i, a in enumerate(archs)]
    print(f"[mock] {len(runs)} runs -> {log_dir.resolve()}")
    print(f"[mock] backfilling {args.backfill} episodes per run ...")
    for _ in range(args.backfill):
        for r in runs:
            r.run_episode()
    print("[mock] live streaming (Ctrl+C to stop)")

    try:
        while True:
            t0 = time.perf_counter()
            for r in runs:
                r.run_episode()
            if args.episodes and runs[0].episode >= args.episodes:
                break
            if runs[0].episode % 25 == 0:
                print("[mock] " + "  ".join(f"{r.a.name}: ep {r.episode}" for r in runs))
            time.sleep(max(0.0, 1.0 / args.rate - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        for r in runs:
            r.tl.close()
        print("[mock] stopped")


if __name__ == "__main__":
    main()
