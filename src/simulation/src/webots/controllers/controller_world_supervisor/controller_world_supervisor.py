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

checkpoint_positions = [
    [-0.314494, -2.47211, 0.0391],
    [1.11162, -2.56708, 0.0391],
    [2.54552, -2.27446, 0.0391],
    [3.58779, -1.38814, 0.0391],
    [3.58016, -0.0800134, 0.0391],
    [3.23981, 1.26309, 0.0391],
    [2.8261, 1.99783, 0.0391],
    [3.18851, 2.71151, 0.0391],
    [3.6475, 4.09688, 0.0391],
    [3.1775, 4.44688, 0.0391],
    [2.58692, 4.5394, 0.0391],
    [1.52457, 4.3991, 0.0391],
    [0.659969, 3.57074, 0.0391],
    [0.000799585, 2.90417, 0.0391],
    [0.0727115, 1.81299, 0.0391],
    [0.788956, 1.22248, 0.0391],
    [1.24749, 0.288391, 0.0391],
    [0.88749, -0.281609, 0.0391],
    [0.0789172, -0.557653, 0.0391],
    [-0.832859, -0.484867, 0.0391],
    [-1.79723, 0.408769, 0.0391],
    [-1.7446, 1.3386, 0.0391],
    [-1.92104, 2.72452, 0.0391],
    [-2.96264, 2.96666, 0.0391],
    [-4.19027, 2.74619, 0.0391],
    [-4.34725, 1.7503, 0.0391],
    [-4.26858, 0.259482, 0.0391],
    [-4.20936, -1.06968, 0.0391],
    [-4.0021, -2.35518, 0.0391],
    [-2.89371, -2.49154, 0.0391],
    [-2.01029, -2.51669, 0.0391],
]

simulation_rank = get_simulation_rank()

class RaceTracker:
    """Tracks (x, y), current checkpoint, n_lap, and unwrapped progress for ALL cars (RL + bots)."""
    def __init__(self, total_cars: int, checkpoints_list):
        self.total_cars = total_cars
        # Extract [N_cp, 2] array of (x0, y0) from Checkpoint objects
        self.cp_xy = np.array([[cp.x0, cp.y0] for cp in checkpoints_list], dtype=np.float32)
        self.nb_cp = len(self.cp_xy)
        self.pos_xy = np.zeros((total_cars, 2), dtype=np.float32)
        self.cp_idx = np.zeros(total_cars, dtype=np.int32)
        self.n_lap = np.zeros(total_cars, dtype=np.int32)
        self.progress = np.zeros(total_cars, dtype=np.int32)

    def reset_car(self, car_idx: int):
        node = supervisor.getFromDef(f"TT02_{car_idx}")
        if node is not None:
            x, y, _ = node.getField("translation").getSFVec3f()
            self.pos_xy[car_idx] = [x, y]
            dists = np.linalg.norm(self.cp_xy - self.pos_xy[car_idx], axis=1)
            self.cp_idx[car_idx] = int(np.argmin(dists))
        self.n_lap[car_idx] = 0
        self.progress[car_idx] = self.cp_idx[car_idx]

    def update_all(self):
        for j in range(self.total_cars):
            node = supervisor.getFromDef(f"TT02_{j}")
            if node is None:
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
            self.progress[j] = self.n_lap[j] * self.nb_cp + new_cp

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
            trans = self.checkpoint_manager.getTranslation()
            self._last_cp_pos = np.array([trans[0], trans[1]], dtype=np.float32)
            rot = self.checkpoint_manager.getRotation()

            self.translation_field.setSFVec3f(trans)
            self.rotation_field.setSFRotation(rot)
            self.checkpoint_manager.update()
            self.tracker.reset_car(self.vehicle_rank)

            vehicle.resetPhysics()
            #self.log.info("vehicle reset done")
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
        ego_prog = self.tracker.progress[self.vehicle_rank]
    
        b_opp = sum(
            1 for j in range(self.tracker.total_cars)
            if j != self.vehicle_rank and ego_prog > self.tracker.progress[j]
        )
        f_opp = sum(
            1 for j in range(self.tracker.total_cars)
            if j != self.vehicle_rank and ego_prog < self.tracker.progress[j]
        )
        return float(c.overtaking_factor * (((b_opp - f_opp) / total_opp) ** 3))
             

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
                reward = np.float32(-0.5)
                done = np.bool_(c.respawn_on_crash)
        elif b_past_checkpoint:
            if self.next_cp_idx < last_cp_idx:
                self.n_lap += 1
            reward = np.float32(self.speed_reward() + self.overtook())
            done = np.False_
        else:
            reward = np.float32(0.05)
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
    tracker = RaceTracker(c.n_vehicles + c.n_stupid_vehicles, envs[0].checkpoints)
    for e in envs:
        e.tracker = tracker
    for i, e in enumerate(envs):
        e.reset(i)
    
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

    # We also need to reset the position of the stupid vehicles at t = 0
    for i in range(c.n_vehicles, c.n_vehicles + c.n_stupid_vehicles):
        (
            supervisor.getFromDef(f"TT02_{i}")
            .getField("translation")
            .setSFVec3f(checkpoint_positions[i])
        )

    last_moved = np.zeros(c.n_stupid_vehicles)

    while supervisor.step() != -1:
        tracker.update_all()
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
            if not e.send_to_server(obs, reward, done, truncated):
                return

            #e.log.info("finished sending everything")
            if done:
                info = e.reset()

        for i in range(c.n_stupid_vehicles):
            bot_node = supervisor.getFromDef(f"TT02_{c.n_vehicles + i}")
            tr_field = bot_node.getField("translation")
            vel = bot_node.getVelocity()  # [vx, vy, vz, wx, wy, wz]
            speed = np.hypot(vel[0], vel[1])

            if speed >= 0.1:
                last_moved[i] = supervisor.getTime()

            if supervisor.getTime() - last_moved[i] > 3.0:
                envs[0].log.info("resetting bot position because stuck for > 3s")
                tr_field.setSFVec3f(checkpoint_positions[i])
                bot_node.resetPhysics()
                last_moved[i] = supervisor.getTime()
                tracker.reset_car(c.n_vehicles + i)


if __name__ == "__main__":
    main()
