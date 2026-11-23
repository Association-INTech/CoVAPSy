import logging
import os
import numpy as np
from controller import Supervisor

from simulation import config as c
from utils.fifo import FifoClosedError, read_exact
from utils.simulation_rank import get_simulation_rank

# I cannot use relative imports because the file is directly launched by webots
from webots.controllers.controller_world_supervisor import (
    CheckpointManager,
    tracks,
)

supervisor = Supervisor()

simulation_rank = get_simulation_rank()

class RaceTracker:
    """Tracks (x, y), current checkpoint, n_lap, and offset-free progress for ALL cars (RL + bots)."""
    def __init__(self, total_cars: int, checkpoints_list):
        self.total_cars = total_cars
        # Extract [N_cp, 2] array of (x0, y0) from Checkpoint objects
        self.cp_xy = np.array([[cp.x0, cp.y0] for cp in checkpoints_list], dtype=np.float32)
        self.nb_cp = len(self.cp_xy)
        self.pos_xy = np.zeros((total_cars, 2), dtype=np.float32)
        self.cp_idx = np.zeros(total_cars, dtype=np.int32)
        self.start_cp = np.zeros(total_cars, dtype=np.int32)
        self.n_lap = np.zeros(total_cars, dtype=np.int32)
        self.progress = np.zeros(total_cars, dtype=np.int32)
        self.step_delta = np.zeros(total_cars, dtype=np.int32)
        self.cum_progress = np.zeros(total_cars, dtype=np.int64)

    def reset_car(self, car_idx: int, cp_idx: int | None = None):
        node = supervisor.getFromDef(f"TT02_{car_idx}")
        if node is not None:
            x, y, _ = node.getField("translation").getSFVec3f()
            self.pos_xy[car_idx] = [x, y]
        if cp_idx is not None:
            cp0 = int(cp_idx) % self.nb_cp
        else:
            dists = np.linalg.norm(self.cp_xy - self.pos_xy[car_idx], axis=1)
            cp0 = int(np.argmin(dists))
        self.cp_idx[car_idx] = cp0
        self.start_cp[car_idx] = cp0
        self.n_lap[car_idx] = 0
        self.progress[car_idx] = 0
        self.step_delta[car_idx] = 0

    def update_all(self):
        for j in range(self.total_cars):
            node = supervisor.getFromDef(f"TT02_{j}")
            if node is None:
                self.step_delta[j] = 0
                continue
            x, y, _ = node.getField("translation").getSFVec3f()
            self.pos_xy[j] = [x, y]
            dists = np.linalg.norm(self.cp_xy - self.pos_xy[j], axis=1)
            new_cp = int(np.argmin(dists))
            prev_cp = self.cp_idx[j]

            # Detect finish-line wrap-around (e.g., from CP 28..30 -> CP 0..3)
            if prev_cp >= self.nb_cp - 5 and new_cp <= 4:
                self.n_lap[j] += 1
            elif prev_cp <= 4 and new_cp >= self.nb_cp - 5:
                self.n_lap[j] -= 1

            self.cp_idx[j] = new_cp
            new_progress = self.n_lap[j] * self.nb_cp + new_cp - self.start_cp[j]
            delta = int(new_progress - self.progress[j])
            self.step_delta[j] = delta
            self.cum_progress[j] += delta
            self.progress[j] = new_progress


def spawn_bot_at_cp(car_idx: int, cp_idx: int, track_cps, tracker: RaceTracker) -> None:
    """Spawn or respawn a bot car on a track checkpoint with matching heading."""
    bot_node = supervisor.getFromDef(f"TT02_{car_idx}")
    if bot_node is None:
        return
    idx = int(cp_idx) % len(track_cps)
    cp = track_cps[idx]
    bot_node.getField("translation").setSFVec3f([cp.x0, cp.y0, 0.0391])
    bot_node.getField("rotation").setSFRotation([0, 0, 1, cp.theta])
    bot_node.resetPhysics()
    tracker.reset_car(car_idx, cp_idx=idx)

class WebotsVehicleManager:
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, vehicle_rank: int):
        self.vehicle_rank = vehicle_rank
        map_idx = c.map_id if getattr(c, "map_id", None) is not None else (simulation_rank % c.n_map)
        self.checkpoint_manager = CheckpointManager(
            supervisor, tracks[map_idx], vehicle_rank
        )
        self.checkpoints = self.checkpoint_manager.checkpoints
        self.next_cp_idx = self.checkpoint_manager.next_checkpoint
        self.last_cp = self.checkpoint_manager.checkpoints[max((self.checkpoint_manager.next_checkpoint - 1), 0)]

        self.v_min = 1
        self.v_max = 9
        basicTimeStep = int(supervisor.getBasicTimeStep())
        self.sensorTime = basicTimeStep // 4
        self._last_cp_pos = np.array([self.last_cp.x0, self.last_cp.y0])
        self._last_cp_time = 0.0
        self.n_lap = 0
        self.progress = self.n_lap * len(self.checkpoints) + self.next_cp_idx
        self._opp_cum_at_reset: np.ndarray | None = None
        self._prev_steer: float | None = None

        # negative value so that the first reset is not skipped
        self.last_reset = -1e6

        self.simulation_rank = get_simulation_rank()
        fifo_dir = c.FIFO_DIR
        os.makedirs(fifo_dir, exist_ok=True)
        self.handler = logging.FileHandler(
            f"{fifo_dir}/vehicle_{self.simulation_rank}_{self.vehicle_rank}.log"
        )
        self.handler.setFormatter(c.FORMATTER)
        self.log = logging.getLogger(
            f"SUPERVISOR_{self.simulation_rank}_{self.vehicle_rank}"
        )
        self.log.setLevel(level=c.LOG_LEVEL)
        self.log.addHandler(self.handler)

        self.log.debug("Connection to the vehicle")
        self.fifo_r = open(
            f"{fifo_dir}/{self.simulation_rank}_{self.vehicle_rank}tosupervisor.pipe",
            "rb",
        )
        self.log.debug("Connection to the server")
        self.fifo_w = open(
            f"{fifo_dir}/{simulation_rank}_{vehicle_rank}toserver.pipe", "wb"
        )

        self.translation_field = supervisor.getFromDef(
            f"TT02_{self.vehicle_rank}"
        ).getField("translation")  # may cause access issues ...
        self.rotation_field = supervisor.getFromDef(
            f"TT02_{self.vehicle_rank}"
        ).getField("rotation")  # may cause access issues ...

    # returns the lidar data of all vehicles
    def observe(self):
        # gets from Vehicle
        #self.log.info(f"Waiting for obs from VEHICLE_{self.simulation_rank}_{self.vehicle_rank}")
        obs = np.frombuffer(
            read_exact(
                self.fifo_r,
                np.dtype(np.float32).itemsize * c.obs_size,
                "Observation from vehicle controller",
            ),
            dtype=np.float32,
        )
        #self.log.debug(f"observing {obs=}")
        return obs

    # reset the gym environment reset
    def reset(self, seed=None):
        self._prev_steer = None
        pos = self.translation_field.getSFVec3f()
        self._last_cp_pos = np.array([pos[0], pos[1]], dtype=np.float32)
        self._last_cp_time = float(supervisor.getTime())
        #self.log.info("reseting vehicle")
        # this has to be done otherwise thec cars will shiver for a while sometimes when respawning and idk why
        if supervisor.getTime() - self.last_reset >= 1e-1:
            #self.log.debug("getting info from vehicle")
            self.last_reset = supervisor.getTime()

            vehicle = supervisor.getFromDef(f"TT02_{self.vehicle_rank}")
            #self.log.debug("resetting vehicle physics")
            self.checkpoint_manager.reset(seed)
            spawn_cp = self.checkpoint_manager.next_checkpoint
            trans = self.checkpoint_manager.getTranslation()
            self._last_cp_pos = np.array([trans[0], trans[1]], dtype=np.float32)
            rot = self.checkpoint_manager.getRotation()

            self.translation_field.setSFVec3f(trans)
            self.rotation_field.setSFRotation(rot)
            self.checkpoint_manager.update()
            self.tracker.reset_car(self.vehicle_rank, cp_idx=spawn_cp)

            vehicle.resetPhysics()
            #self.log.info("vehicle reset done")
        if hasattr(self, "tracker"):
            self._opp_cum_at_reset = self.tracker.cum_progress.copy()
        info = {}
        return info

    def speed_reward(self):
        pos = self.translation_field.getSFVec3f()
        cur_pos = np.array([pos[0], pos[1]], dtype=np.float32)
        t_now = float(supervisor.getTime())

        dt_sector = max(t_now - self._last_cp_time, 0.025)  # At least 1 sim step (25 ms)
        dist_sector = float(np.linalg.norm(cur_pos - self._last_cp_pos))
        v_sector = dist_sector / dt_sector  # Real physical average speed (m/s) across the sector!

        # Update memory for the next checkpoint
        self._last_cp_pos = cur_pos
        self._last_cp_time = t_now

        # Quadratic bonus on real achieved sector speed (with v_ref ≈ 1.25 m/s)
        v_ref = c.target_speed
        lambda_v = c.speed_factor
        reward = np.float32(1.0 + lambda_v * ((v_sector / v_ref) ** 2))
        return reward

    def closest_opp(self) -> float:
        if self.tracker.total_cars <= 1:
            return 999.0
        ego_xy = self.tracker.pos_xy[self.vehicle_rank]
        opp_dists = [
            float(np.linalg.norm(ego_xy - self.tracker.pos_xy[j]))
            for j in range(self.tracker.total_cars)
            if j != self.vehicle_rank
        ]
        return min(opp_dists) if opp_dists else 999.0

    def overtook(self) -> float:
        total_opp = self.tracker.total_cars - 1
        if total_opp <= 0:
            return 0.0
        ego_prog = float(self.tracker.progress[self.vehicle_rank])
        base = (
            self._opp_cum_at_reset
            if self._opp_cum_at_reset is not None
            else np.zeros(self.tracker.total_cars, dtype=np.int64)
        )
        opp_progs = [
            float(self.tracker.cum_progress[j] - base[j])
            for j in range(self.tracker.total_cars)
            if j != self.vehicle_rank
        ]
        mean_opp_prog = float(np.mean(opp_progs))
        rel_lead = max(0.0, ego_prog - mean_opp_prog)
        return float(c.overtaking_factor * np.tanh(rel_lead / 5.0))

    def steer_penalty(self, obs: np.ndarray) -> float:
        """Penalty proportional to |delta_t - delta_{t-1}| (rad/step), scaled by c.steer_reward."""
        if "Steer_Cmd" not in c.active_obs:
            return 0.0
        steer_index = c.active_obs.index("Steer_Cmd")
        steer_start = sum(c.OBS_SPECS[key] for key in c.active_obs[:steer_index])
        cur_steer = float(obs[steer_start])
        if self._prev_steer is None:
            self._prev_steer = cur_steer
            return 0.0
        delta_steer = abs(cur_steer - self._prev_steer)
        self._prev_steer = cur_steer
        return float(c.steer_reward * (delta_steer/0.4)**2)

    # step function of the gym environment
    def step(self):
        # we should add a beacon sensor pointing upwards to detect the beacon
        #self.log.debug("getting observation")
        obs = self.observe()
        #self.log.info(f"observed {obs=}")
        collision_index = c.active_obs.index("Collision")
        collision_start = sum(
            c.OBS_SPECS[key] for key in c.active_obs[:collision_index]
        )
        collision_end = collision_start + c.OBS_SPECS["Collision"]
        sensor_data = obs[collision_start:collision_end]
        reward = 0
        done = np.False_
        truncated = np.False_

        x, y, z = self.translation_field.getSFVec3f()

        last_cp_idx = self.next_cp_idx
        b_past_checkpoint = self.checkpoint_manager.update(x, y)
        self.next_cp_idx = self.checkpoint_manager.next_checkpoint

        (b_collided,) = sensor_data
        d_opp = self.closest_opp()
        steer_pen = self.steer_penalty(obs)
        if z < -10:
            reward = np.float32(0.0)
            done = np.True_
        elif b_collided:
            if d_opp < 0.45:
                # Bumped an opponent car
                reward = np.float32(-0.2)
                done = np.False_
            else:
                # Crashed into track wall
                reward = np.float32(-0.8)
                done = np.bool_(c.respawn_on_crash)
        elif b_past_checkpoint:
            if self.next_cp_idx < last_cp_idx:
                self.n_lap += 1
            reward = np.float32(self.speed_reward() + self.overtook() - steer_pen)
            done = np.False_
        else:
            reward = np.float32(0.05 - steer_pen)
            done = np.False_

        return (
            obs, 
            reward, 
            done, 
            truncated, 
            {},
        )

    def send_to_server(self, *values: np.ndarray | np.generic) -> bool:
        try:
            for value in values:
                self.fifo_w.write(value.tobytes())
            self.fifo_w.flush()
        except BrokenPipeError:
            self.log.warning(
                "Vehicle environment closed its FIFO; stopping Webots supervisor"
            )
            return False
        return True


def main():
    envs = [WebotsVehicleManager(i) for i in range(c.n_vehicles)]
    track_cps = envs[0].checkpoints
    n_cp = len(track_cps)
    tracker = RaceTracker(c.n_vehicles + c.n_stupid_vehicles, track_cps)
    for e in envs:
        e.tracker = tracker
    for i, e in enumerate(envs):
        e.reset(i)

    # Spawn stupid vehicles evenly spaced around the active track (away from CP 0)
    for k in range(c.n_stupid_vehicles):
        car_idx = c.n_vehicles + k
        cp_idx = int(((k + 1) * n_cp) // (c.n_stupid_vehicles + 1)) % n_cp
        spawn_bot_at_cp(car_idx, cp_idx, track_cps, tracker)
    for e in envs:
        e._opp_cum_at_reset = tracker.cum_progress.copy()

    # At t = 0, we need to step the supervisor once to make sure
    # that the vehicles are in the right position before starting the simulation
    supervisor.step()
    for e in envs:
        try:
            initial_obs = e.observe()
        except FifoClosedError as exc:
            e.log.warning("%s; stopping Webots supervisor", exc)
            return
        if not e.send_to_server(initial_obs):
            return

    last_moved = np.full(c.n_stupid_vehicles, supervisor.getTime(), dtype=np.float64)

    while supervisor.step() != -1:
        tracker.update_all()
        bot_deltas = tracker.step_delta[
            c.n_vehicles : c.n_vehicles + c.n_stupid_vehicles
        ].astype(np.float32)
        for i, e in enumerate(envs):
            #e.log.info("starting new step")

            try:
                (
                    obs,
                    reward,
                    done,
                    truncated,
                    info,
                ) = e.step()
            except FifoClosedError as exc:
                e.log.warning("%s; stopping Webots supervisor", exc)
                return

            #e.log.info(f"sending {obs=}")
            if c.n_stupid_vehicles > 0:
                ok = e.send_to_server(obs, reward, done, truncated, bot_deltas)
            else:
                ok = e.send_to_server(obs, reward, done, truncated)
            if not ok:
                return

            #e.log.info("finished sending everything")
            if done:
                info = e.reset()

        for i in range(c.n_stupid_vehicles):
            car_idx = c.n_vehicles + i
            bot_node = supervisor.getFromDef(f"TT02_{car_idx}")
            if bot_node is None:
                continue
            vel = bot_node.getVelocity()  # [vx, vy, vz, wx, wy, wz]
            speed = np.hypot(vel[0], vel[1])

            if speed >= 0.1:
                last_moved[i] = supervisor.getTime()

            if supervisor.getTime() - last_moved[i] > 3.0:
                envs[0].log.info("resetting bot position because stuck for > 3s")
                occupied = {
                    int(tracker.cp_idx[j])
                    for j in range(tracker.total_cars)
                    if j != car_idx
                }
                candidates = [
                    idx
                    for idx in range(n_cp)
                    if all(min((idx - o) % n_cp, (o - idx) % n_cp) >= 2 for o in occupied)
                ]
                respawn_cp = (
                    int(np.random.choice(candidates))
                    if candidates
                    else (int(tracker.cp_idx[car_idx]) + 1) % n_cp
                )
                spawn_bot_at_cp(car_idx, respawn_cp, track_cps, tracker)
                last_moved[i] = supervisor.getTime()


if __name__ == "__main__":
    main()
