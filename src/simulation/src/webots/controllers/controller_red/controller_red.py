from typing import cast
import numpy as np
from controller import Camera, Lidar
from controller.touch_sensor import TouchSensor
from vehicle import Driver

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTime = basicTimeStep // 4

lidar = Lidar("Hokuyo")
lidar.enable(sensorTime)
lidar.enablePointCloud()

camera = cast(Camera, driver.getDevice("RASPI_Camera_V2"))
camera.enable(sensorTime)

touch_sensor = cast(TouchSensor, driver.getDevice("touch_sensor"))
touch_sensor.enable(sensorTime)

# Kinematic Bicycle & MPC Parameters
L = 0.257          # Wheelbase (m)
DT = 0.025         # 25 ms step
H = 12             # Prediction horizon: 12 steps = 0.30 s ahead
MAX_STEER = 0.28   # rad
V_MIN_MPS = 0.85   # m/s (3.06 km/h)
V_MAX_MPS = 1.45   # m/s (5.22 km/h) - competitive pace but overtakable by 22s ResNetMemo!
CAR_RADIUS = 0.22  # m safety clearance from walls & other cars

# Bank of K = 63 candidate trajectories (21 steering angles x 3 speed ratios)
steer_grid = np.linspace(-MAX_STEER, MAX_STEER, 21, dtype=np.float32)
speed_ratios = np.array([0.65, 0.85, 1.00], dtype=np.float32)
STEER_CAND, SPEED_RATIO_CAND = np.meshgrid(steer_grid, speed_ratios, indexing="ij")
STEER_CAND = STEER_CAND.reshape(-1)
SPEED_RATIO_CAND = SPEED_RATIO_CAND.reshape(-1)

backwards_duration = 1500  # ms
stop_duration = 2000       # ms
death_count = 0
prev_steer = 0.0

driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(0.0)


def backwards(camera_data: np.ndarray):
    for _ in range(backwards_duration // basicTimeStep):
        avg_color = np.mean(camera_data, axis=0) / 255.0
        angle = float(-0.4 * avg_color[0] + 0.4 * avg_color[1])
        driver.setCruisingSpeed(-2.0)
        driver.setSteeringAngle(angle)
        driver.step()

    for _ in range(6):
        driver.setCruisingSpeed(2.0)
        driver.setSteeringAngle(0.0)
        driver.step()


def stop():
    driver.setCruisingSpeed(0.0)
    driver.setSteeringAngle(0.0)
    for _ in range(stop_duration // basicTimeStep):
        driver.step()


def mpc_step(lidar_ranges: np.ndarray, prev_delta: float) -> tuple[float, float]:
    n_rays = lidar_ranges.size
    # In Webots Hokuyo: index 0 is left (+135 deg), index -1 is right (-135 deg)
    # In Webots Driver.setSteeringAngle: positive angle steers RIGHT (-y), negative steers LEFT (+y)
    # So we define +y as RIGHT to match Webots steering sign directly!
    angles = np.linspace(-0.75 * np.pi, 0.75 * np.pi, n_rays, dtype=np.float32)

    # 1. Downsample LiDAR to M = 45 obstacle points
    stride = max(1, n_rays // 45)
    r_sub = np.clip(lidar_ranges[::stride], 0.05, 10.0)
    ang_sub = angles[::stride]
    obs_pts = np.stack([r_sub * np.cos(ang_sub), r_sub * np.sin(ang_sub)], axis=-1)

    # 2. Find local lookahead goal inside the widest smoothed gap in [-80 deg, +80 deg]
    front_mask = np.abs(angles) <= np.deg2rad(80.0)
    r_front = np.where(front_mask, np.clip(lidar_ranges, 0.0, 8.0), 0.0)
    kernel = np.ones(15, dtype=np.float32) / 15.0
    r_smooth = np.convolve(r_front, kernel, mode="same")
    best_idx = int(np.argmax(r_smooth))
    goal_dist = float(np.clip(r_smooth[best_idx] - 0.35, 0.4, 2.5))
    goal_angle = float(angles[best_idx])
    goal_x = goal_dist * np.cos(goal_angle)
    goal_y = goal_dist * np.sin(goal_angle)

    # 3. Adapt reference speed to forward clearance
    v_ref = np.clip(0.55 + 0.45 * goal_dist, V_MIN_MPS, V_MAX_MPS)
    v_cand = v_ref * SPEED_RATIO_CAND
    delta_cand = STEER_CAND

    # 4. Roll out K=63 kinematic bicycle trajectories over H=12 steps
    K = v_cand.size
    x = np.zeros(K, dtype=np.float32)
    y = np.zeros(K, dtype=np.float32)
    psi = np.zeros(K, dtype=np.float32)
    traj_x = np.empty((K, H), dtype=np.float32)
    traj_y = np.empty((K, H), dtype=np.float32)
    tan_delta = np.tan(delta_cand)

    for k in range(H):
        x = x + v_cand * np.cos(psi) * DT
        y = y + v_cand * np.sin(psi) * DT
        psi = psi + (v_cand / L) * tan_delta * DT
        traj_x[:, k] = x
        traj_y[:, k] = y

    traj_pts = np.stack([traj_x, traj_y], axis=-1)  # [K, H, 2]

    # 5. Minimum distance to LiDAR obstacles along each trajectory
    diff = traj_pts[:, :, None, :] - obs_pts[None, None, :, :]
    min_dist_per_traj = np.sqrt(np.min(np.sum(diff * diff, axis=-1), axis=(1, 2)))

    # 6. Evaluate MPC Cost Function J(U)
    goal_err_sq = (traj_x[:, -1] - goal_x) ** 2 + (traj_y[:, -1] - goal_y) ** 2
    obs_cost = 6.0 * np.exp(-(min_dist_per_traj ** 2) / (2.0 * (0.28 ** 2)))
    collision_penalty = np.where(min_dist_per_traj < CAR_RADIUS, 1e4, 0.0)
    speed_cost = 0.6 * ((V_MAX_MPS - v_cand) ** 2)
    smooth_cost = 1.8 * ((delta_cand - prev_delta) ** 2) + 0.3 * (delta_cand ** 2)

    total_cost = goal_err_sq + obs_cost + collision_penalty + speed_cost + smooth_cost
    best_k = int(np.argmin(total_cost))

    return float(delta_cand[best_k]), float(v_cand[best_k] * 3.6)


while driver.step() != -1:
    lidar_data = np.asarray(lidar.getRangeImage(), dtype=np.float32)
    lidar_data = np.nan_to_num(lidar_data, nan=0.0, posinf=30.0, neginf=0.0)
    lidar_data[lidar_data <= 0.0] = 30.0
    camera_data = np.nan_to_num(camera.getImageArray(), nan=0.0, posinf=30.0).squeeze()
    sensor_data = touch_sensor.getValue()

    if sensor_data == 1:
        death_count += 1
        if death_count < 3:
            backwards(camera_data)
        else:
            death_count = 0
            stop()
        prev_steer = 0.0
        continue

    angle, speed_kmh = mpc_step(lidar_data, prev_steer)
    prev_steer = angle

    driver.setCruisingSpeed(speed_kmh)
    driver.setSteeringAngle(angle)