# CoVAPsy Simulation & RL Training Architecture

End-to-end documentation for the Webots simulation detailing multi-process FIFO IPC, observation contracts, extractors/policies, and the ONNX deployment bridge.
Most of the theory comes from papers whose links live inside the `literature` folder.

## Table of Contents
- [System Architecture](#system-architecture)
- [IPC & Observation Space](#ipc-obs)
- [Configuration & Environment Variables](#config)
- [Model Architectures & Policy Algorithms](#model-architectures--policy)
- [Anatomy of an Extractor](#extractors)
- [Anatomy of a Controller](#controller)
- [Rewards & Adversarial Behaviour](#reward--multi-ag)
- [ONNX Export & Querying ROS Topics](#onnx--deployment)
- [Running the Model on the Car](#running)
- [Webots Limits & Genesis World](#simulator)
- [E2E Simulation Limits & How MPC Tackles Sim2Real](#Sim2R-challenges)
- [Crash Course to Dive In](#crash)

---

<a id="system-architecture"></a>
## System Architecture

The whole architecture relies on three main processes communicating through Linux named FIFO pipes. It is at the core of every RL training loop.

![System Architecture](image.png)

- **Supervisor:** Acts as an omniscient referee able to see the position, speed, and state of each car in the world. It computes rewards and termination flags based on the car state and writes them into a pipe to the server.
- **Server (`VehicleEnv`):** Acts as the brain of the car. Each RL car possesses its own `VehicleEnv` worker inside `SubprocVecEnv`. It reads observations from the pipe and passes them to Stable-Baselines3, which feeds them through the feature extractor and policy heads to output two actions: `speed` and `steer`. Those actions are written back to a pipe for the car to read.
- **Car Controller (`controller_vehicle_driver.py`):** Reads those commands and acts as the body. It sends its onboard sensor data to the supervisor and executes the actions received from `VehicleEnv`.

---

<a id="ipc-obs"></a>
## IPC & Observations

These processes communicate through Linux `mkfifo` pipes located at `/tmp/autotech/{MODEL_ARCH}/{sim_rank}_{vehicle_rank}{direction}.pipe` (`serverto...`, `...tosupervisor`, `...toserver`).

Each environment maintains an observation dictionary of values clamped to `[-30, 30]`.

**`OBS_SPECS` Shapes** (with `axis 0 = time`, `axis 1 = sensor data`):

- **Camera (`Cam`):** `(c.context_size, hor_res * ver_res)` array
  <details>
  <summary>How the shape is computed</summary>

  1. `(c.context_size, camera_horizontal_resolution, camera_vertical_resolution, 3)`
  2. Flattened and transposed to `(c.context_size, 3, hor_res * ver_res)`
  3. Simplified to a 1D vector `(c.context_size, hor_res * ver_res)` keeping the dominant color (`red: -1`, `green: 1`, `blue: 0`).

  </details>
- **LiDAR (`Lidar`):** `(c.context_size, lidar_res)` — range distances in meters (`[0.0, 30.0]`).
- **Touch Sensor (`Collision`):** `(c.context_size, nb_sensors)` — binary contact flags.
- **`Speed_Cmd`, `Steer_Cmd`, `Wheel_Speed`:** `(c.context_size, 1)` — scalar commands/measurements.
- **Acceleration (`Accel`), Internal Yaw Rate (`Gyro`):** `(c.context_size, 3)` — 3-axis IMU measurements.

 **Notes on Pipe Serialization:**
 - All values (even booleans) are cast to `float32` so the FIFO pipes carry a single uniform data type.
 - What travels through a pipe is strictly a flat `float32` 1D array, never a Python `dict`. The `dict` is only an in-memory structure so we can toggle active sensors without hardcoding pipe byte offsets.
 - `OBS_SPECS` and `obs_indexes` map each slice of the flat byte stream. Therefore, observations are **always** unpacked immediately **after** reading the pipe and packed immediately **before** writing to it.

---

<a id="config"></a>
## Configuration & Environment Variables

`config.py` holds all runtime specifications for the simulation—most notably which observations are **active** (`active_obs` transmitted through the pipes), which extractor architecture (`AUTOTECH_ARCH`) is active, and the hyperparameters passed to the RL policy algorithm.

To launch a simulation, two components operate at different stages:

- **`controller_world_init.py`:** Runs once at $t = 0$ to inject robot nodes into the Webots scene tree—spawning the bodies of the cars and `WorldSupervisor`, and assigning a controller to each car. Because controllers are assigned per node, you can mix different controllers inside a single run and race them head-to-head (for example, a trained RL model vs. an MPC bot vs. a new RL agent). The supervisor then instantiates a **`WebotsVehicleManager`** for each RL car as a god-mode judge tracking its position, speed, and checkpoint progress.
- **`launch_train_multiprocessing.py`:** Connects SB3's `SubprocVecEnv` to the Webots instances so each `VehicleEnv(sr, vr)` worker feeds into the centralized RL loop. `SubprocVecEnv` batches observations across all workers, runs the PyTorch policy + feature extractor forward pass on the GPU, and writes the resulting actions back to each `serverto{sr}_{vr}.pipe`.

---

<a id="model-architectures--policy"></a>
## Model Architectures & Policy Algorithms

Currently the simulation supports two SB3 algorithms (`PPO` and `SAC`), though any off-the-shelf SB3 policy compatible with `MultiInputPolicy` can be plugged in. For a full theoretical treatment, see [OpenAI Spinning Up — Kinds of RL Algorithms](https://spinningup.openai.com/en/latest/spinningup/rl_intro2.html).

In brief, a policy maps the current embedded state $\phi(s_t)$ (which already packs `context_size` frames of recent sensor and command history) to an action distribution that maximizes expected future discounted rewards. The raw observation dictionary (`Lidar`, `Cam`, `Speed_Cmd`, `Steer_Cmd`) is first compressed by our feature extractor into a single 1D latent state vector $\phi(s_t)$.

Under the hood, Stable-Baselines3 wires a three-stage `MultiInputPolicy`:

1. **Embedded Latent State:** $\phi(s_t) = \text{features\_extractor}(s_t)$
   - Compresses multi-modal temporal tensors into a single 1D latent vector of shape `[B, nb_features]`.
2. **MLP Extractor (`net_arch=[64, 64]`):**
   - **Actor branch:** `latent_pi = policy_net(ϕ(s_t))` $\to$ `[B, 64]`
   - **Critic branch:** `latent_vf = value_net(ϕ(s_t))` $\to$ `[B, 64]`
3. **Output Heads (Algorithm-Specific):**
   - **PPO (`MultiDiscrete([16, 16])`):** `action_net(latent_pi)` $\to$ `32` logits (`16` steer bins + `16` speed bins).
   - **SAC (`Box([-1, 1])`):** `actor.mu(latent_pi)` $\to$ `2` continuous action means + `log_std`.

> **Why there are three networks:** Alongside the feature extractor we design, SB3 simultaneously optimizes both the **Actor** (choosing actions) and the **Critic** (estimating state/action value). Because we don't know the true optimal return beforehand, the Critic learns to approximate it so the Actor has a baseline to judge whether an action performed better or worse than expected.

---

<a id="extractors"></a>
## Anatomy of an Extractor

Here is **what needs to exist in an extractor** so you can build your own—for the underlying neural network theory, jump into the `literature` folder!

1. **Attributes:** An extractor must declare `features_dim` (output vector size) and inspect `observation_space` to know which active sensors to read and how many past frames (`context_size`) are buffered.
2. **Backbone (`self.net` inside `__init__`):** Typically defined as an `nn.Sequential` pipeline of 1D convolutions, pooling layers, and normalizations (`GroupNorm` or `Identity`—never `BatchNorm1d` or `Dropout1d`) applied to the spatial inputs.
3. **`forward(self, observations)` Pass:** Takes the observation dictionary, normalizes raw sensor scales so no single input dominates by magnitude alone, runs `self.net`, and optionally concatenates engineered kinematic features (such as speed, steering, or Savitzky-Golay range derivatives) before returning the final `[B, features_dim]` tensor. Any custom layer not native to PyTorch must be defined as an `nn.Module` subclass.

<details>
<summary>Deep Dive: Rationale Behind the ResNet Hyperparameter Tweaks</summary>

Here we dissect `CNN1DResNetExtractorMemory` (`resmemo`) to illustrate the principles above:

- **Channel Stacking & Scalar Separation:** Every channel stacked into the 1D CNN input must share the same spatial width (`1024`), which is why `Lidar` and `Cam` resolutions must match. Duplicating scalar commands (`Speed_Cmd`, `Steer_Cmd`) `1024` times across the spatial axis would waste compute, so scalar history is processed separately in `motion_proj` and concatenated after spatial compression.
- **Spatial vs. Channel Scaling (`[B, 8, 1024]` $\to$ `[B, 128, 16]`):** Track geometry is spatially redundant—keeping `1024` fine angular bins deep into the network wastes capacity. By downsampling spatial width down to `16` angular sectors while expanding feature depth to `128` channels, each sector retains rich semantic context (corners, gaps, walls, opponents) at low latency.
- **Custom `Compressor` Module:** Tackles spatial redundancy right at the stem by combining **AvgPool** (regional profile), **MaxPool** (open gaps/corridors), and **MinPool** (`-MaxPool(-x)`, closest obstacles/walls) to preserve high-contrast boundaries while cutting spatial width by $4\times$ in one step.
- **`ResidualBlock` & `alpha` Anti-Dilution Gate:** Residual skip connections ($y = \mathcal{F}(x) + \alpha x$) prevent vanishing gradients over long training horizons so the policy never stops exploring. Introducing a learnable/scaled bypass factor $\alpha = 0.5$ prevents the raw identity signal from diluting newly learned convolutional features early in training.
- **Kinematic Memory (`motion_proj`):** In `forward()`, past commands and Savitzky-Golay range-rate estimates are extracted, normalized, and projected through a dedicated linear layer (`16`-D) before concatenating with the `64`-D spatial embedding (`80`-D total), ensuring kinematic signals aren't drowned out by spatial features.

</details>

---

<a id="controller"></a>
## Anatomy of a Controller

A Webots controller is the standalone process attached to a robot node in the 3D world. Depending on its role, it follows one of two patterns:

- **RL Training Body (`controller_vehicle_driver.py`):** A stateless bridge that reads virtual sensors (`Lidar`, `Camera`, `TouchSensor`), packs them into `float32` bytes for `WorldSupervisor`, blocks on `serverto{sr}_{vr}.pipe` for the next `(steer, speed)` action, and applies it to `vehicle.Driver`.
- **Standalone / Baseline Bots (`controller_violet.py`):** Self-contained controllers that compute actions locally and implement three behaviors:
  - `backwards()`: Reverses out of a wall after a touch-sensor collision.
  - `stop()`: Freezes the car at `0 km/h` if it gets wedged after repeated collisions (`death_count == 3`), intentionally tripping the supervisor's stuck timer so the car gets teleported and reset instead of looping endlessly against a barrier.
  - `step()`: Computes steering and speed directly from local sensors and sends them to Webots' closed-loop `vehicle.Driver` (which simulates low-level Ackermann steering and motor velocity control).

---

<a id="reward--multi-ag"></a>
## Rewards & Adversarial Behaviour

Here we dissect the referee logic inside `controller_world_supervisor.py`:

- **`WebotsVehicleManager`:** A god-mode judge instantiated for each RL vehicle by the supervisor.
- **`CheckpointManager` (`cp_manager`):** Tracks each car's progress along the closed circuit by monitoring which checkpoint segment it must cross next, and handles clean respawns by resetting target checkpoints after a crash.
- **Reward Shaping:** Computes step rewards from forward checkpoint progress (`delta_cp / time_elapsed`), steering smoothness penalties (`|steer_t - steer_{t-1}|`), crash penalties, and overtaking bonuses.
- **`RaceTracker`:** Shared tracker maintaining unwrapped lap + checkpoint progress (`n_lap * nb_cp + cp_idx`) and $(x, y)$ coordinates for **all** vehicles (both RL agents and opponent bots). This allows `WebotsVehicleManager` to reward net overtakes (`((b_opp - f_opp) / total_opp) ** 3`) and penalize inter-vehicle collisions using exact Euclidean distance between cars.

---

<a id="onnx--deployment"></a>
## ONNX Export & Querying ROS Topics

During training, our model lives inside a heavy Stable-Baselines3 `.zip` checkpoint containing the Actor (`policy_net`), the Critic (`value_net`), and optimizer states. On the physical car (Raspberry Pi), we don't need gradients or value estimates—only fast forward inference.

### 1. Exporting `.zip` $\to$ `.onnx` (`onnx_utils.py` $\to$ `./models`)
`src/simulation/src/utils/onnx_utils.py` handles the conversion into the `./models` directory:
- **`DictOnnxWrapper`:** Extracts only `features_extractor`, `mlp_extractor.policy_net`, and `action_net` from the SB3 policy.
- **Graph Pruning & Naming:** `export_onnx()` traces the model with dummy tensors of shape `(1, c.context_size, c.OBS_SPECS[k])` and exports named inputs (`input_names=list(c.active_obs)`). Any observation key in `active_obs` not consumed by `forward()` (such as `Collision`) is automatically pruned from the ONNX graph.
- **Parity Verification (`test_onnx`):** Compares PyTorch `.eval()` and `.train()` outputs against `onnxruntime.InferenceSession` via MSE loss (confirming `GroupNorm` keeps train/eval behavior identical).

### 2. Hardware Drivers & Sensor Ingestion (`./src/drivers`)
In simulation, observations came from Webots virtual devices (`controller.Lidar`, `controller.Camera`). On the physical car, `src/high_level/src/drivers/` provides the hardware abstraction layer (`Lidar`, `Camera`, `Camera_red_or_green`, `ToF`, `I2CArduino`):
- **LiDAR:** Reads raw distances from the physical RPLiDAR, converts millimeters to meters, replaces `0` / `NaN` dropouts with `30.0 m`, and resizes to match the model's expected `horizontal_size` (e.g., `1024` or `1080`).
- **Camera:** Downsamples the Pi Camera feed into the same 1D color-coded vector (`red -> -1`, `blue -> 0`, `green -> 1`) used in simulation.
- **Rear ToF (`tof.py`):** Runs a daemon thread polling the VL53L1X Time-of-Flight sensor (`tof.distance`) to guard reverse maneuvers.

---

<a id="running"></a>
## Running the Model on the Car

Once the `.onnx` file is placed in `./models`, physical execution splits between **AI inference** (`src/programs/utils/driver.py`) and **vehicle orchestration & safety** (`src/programs/car.py`).

### 1. Inference Bridge (`src/programs/utils/driver.py`)
`Driver` is kept pure Python/NumPy so it can run identically on the Raspberry Pi or inside a Webots evaluation controller:
- **Auto-Detecting Model Generations (`load_model`):** Inspects `len(input_infos[0].shape)` to support all three model eras:
  - **Rank 2 `[B, W]` (`lidar_only`):** Legacy single-scan LiDAR model (`context_size = 1`).
  - **Rank 4 `[B, S, T, W]` (`fused`):** Single-tensor stacked `[LiDAR, Cam]` over `T` frames.
  - **Rank 3 `[B, T, F]` (`dict_inputs`):** Current heterogeneous dictionary architecture (`Lidar`, `Cam`, `Speed_Cmd`, `Steer_Cmd`), allowing scalar commands of width `1` alongside `1024`-ray LiDAR scans.
- **Temporal Context & Action Decoding (`ai_update_lidar_camera`):**
  1. Shifts each buffer in `self.context_dict` (`[:, 1:, :]`) and appends the newest `(1, 1, F)` sensor/command frame.
  2. Runs `self.ai_session.run(None, self.context_dict)` to get the `32` raw action logits (`16` steering bins + `16` speed bins).
  3. Applies temperature-scaled softmax (`Temperature_steer`, `Temperature_vitesse`) and computes the expected value against `ANGLE_LOOKUP` and `SPEED_LOOKUP` for smooth continuous control, saving `self.last_angle` and `self.last_speed` for the next tick's memory.

### 2. Main Control Loop & Safety Guards (`src/programs/car.py`)
`car.py` ties the sensor drivers, AI `Driver`, and low-level actuators together in real time:
1. **Sensor Polling:** Fetches the latest LiDAR scan and camera array from `./src/drivers` and calls `driver.ai_update_lidar_camera(lidar_data, camera_data)` to obtain `(angle, speed)`.
2. **Hardware Safety Overrides (`CrashCar`):** A background thread (`has_Crashed`) continuously checks LiDAR contour penetration (`deadzone = 40 mm`). If a wall impact is detected (`crash_car.crashed == True`), `car.py` overrides the neural network, triggers a reverse recovery routine (checking `self.tof.distance` to avoid backing into a rear wall), and resumes AI control once clear.
3. **Actuation:** Sends the validated steering angle and target speed over `I2CArduino` to the TT-02 steering servo and motor ESC.

---

<a id="simulator"></a>
## Webots Limits & Genesis World

Webots got us up and running with realistic 3D TT-02 physics, LiDAR raycasting, and camera rendering, but scaling RL training on it hits a hard architectural wall:

- **CPU Physics Bottleneck & GPU Starvation:** Webots runs its physics and sensor rendering on the CPU at a fixed `25 ms` step. During `collect_rollouts()`, PyTorch only evaluates tiny batches (`B = n_simulations * n_vehicles`), meaning our GPU spends over 90% of wall-clock time sitting idle waiting for Webots CPU steps and Linux FIFO pipe reads.
- **Multi-Process IPC Overhead:** Because Webots is an external C++ executable that owns the simulation loop, we have to split the brain (`VehicleEnv`), the body (`controller_vehicle_driver.py`), and the referee (`controller_world_supervisor.py`) across separate OS processes connected by three `mkfifo` pipes per car, adding CPU scheduling latency.

### How Genesis World Solves This
Moving to **Genesis World** (a pure-Python, PyTorch-native GPU physics engine) eliminates the external simulator split completely:

| Component | Webots Architecture | Genesis World Architecture |
| :--- | :--- | :--- |
| **Physics & Sensors** | External C++ GUI/headless process on CPU | Native PyTorch tensors running directly on CUDA |
| **Process Topology** | 3 OS processes + 3 FIFO pipes per vehicle | **Single unified `GenesisRaceEnv(gym.Env)`** (no pipes, no `controller_vehicle_driver`, no `WorldSupervisor` split) |
| **Parallelism** | 2–4 instances via `SubprocVecEnv` | Hundreds of vectorized environments stepped in parallel on GPU |
| **Extractors & ONNX** | `CNN1DResNetExtractorMemory` $\to$ `.onnx` | **100% reused as-is** (same `OBS_SPECS` dictionary contract) |

---

<a id="Sim2R-challenges"></a>
## E2E Simulation Limits & How MPC Tackles Sim2Real

Training an end-to-end RL policy purely in simulation exposes several Sim2Real gaps when deploying the `.onnx` model onto the physical TT-02 on the Raspberry Pi:

- **Command vs. Reality Mismatch (Tire Slip):** Currently, `motion_proj` feeds `Speed_Cmd` and `Steer_Cmd` into the network—which is what the policy **asked** for, not what the chassis actually did. Hard braking or tight hairpins cause understeer and wheel lockup that command history alone cannot observe. Fusing real **IMU (`Accel`, `Gyro`)** and **Wheel Odometry** closes this gap.
- **Sensor & Blind-Spot Domain Shift:** Our 1D camera projection (`red -> -1`, `green -> 1`, `blue -> 0`) works cleanly under uniform Webots lighting, whereas real track reflections and shadows introduce noise. Additionally, the Hokuyo LiDAR has a **$90^\circ$ rear blind spot** ($135^\circ \to 225^\circ$), meaning once the car overtakes an opponent directly ahead, that opponent vanishes from LiDAR.
- **Actuation Latency:** On the physical car, ROS2 topic transport, ONNX runtime inference, and ESC/servo response introduce small delays that must stay strictly inside our **`25 ms` real-time budget**.

### How MPC Tackles Sim2Real (and Helps Train Better RL)
**Model Predictive Control (MPC)** uses an explicit kinematic/dynamic bicycle model to simulate candidate steering/throttle sequences over a short lookahead horizon and pick the trajectory that minimizes track error while respecting friction and wall constraints. In our stack, MPC plays two strategic roles:
1. **Competitive Training Sparring Partner:** Replacing naive baseline bots with a fast Shooting-MPC controller gives our RL agents a realistic, high-speed opponent to overtake across all 4 tracks.
2. **Sim2Real Safety & Baseline Bridge:** Because MPC relies on physical vehicle geometry rather than learned visual weights, it transfers reliably to the real car with minimal tuning—serving both as a deterministic benchmark and as a safety fallback while we validate new RL checkpoints on hardware.

---

<a id="crash"></a>
## Crash Course to Dive In

Want to get your hands dirty right away? Here is a map of where everything lives and how to run it.

### 1. Where the Key Files Live
- `src/simulation/src/simulation/config.py` — Central hub: sets `MODEL_ARCH`, `active_obs`, `n_vehicles`, `n_stupid_vehicles`, and `ExtractorClass`.
- `src/simulation/src/simulation/vehicle_env.py` — Gymnasium wrapper (`VehicleEnv`) managing the rolling `(context_size, feature_dim)` buffer and FIFO pipe client.
- `src/simulation/controllers/controller_world_init/` — One-shot instantiator that spawns `WorldSupervisor` and `TT02_i` nodes at $t = 0$.
- `src/simulation/controllers/controller_world_supervisor/` — God-mode track referee (`WebotsVehicleManager`, `CheckpointManager`, `RaceTracker`) computing rewards and resets.
- `src/simulation/controllers/controller_vehicle_driver/` — Onboard sensor-to-pipe and pipe-to-actuator bridge for RL cars.
- `src/simulation/src/extractors/` — Custom PyTorch `BaseFeaturesExtractor` backbones (`res`, `resmemo`, `alex`).
- `src/simulation/scripts/launch_train_multiprocessing.py` — Launches `SubprocVecEnv` + Stable-Baselines3 (`PPO`/`SAC`) and PitWall telemetry logging.

### 2. Essential CLI Commands
```bash
# 1. Start multi-process RL training (select extractor via AUTOTECH_ARCH: res | resmemo | alex)
AUTOTECH_ARCH=resmemo uv run python src/simulation/scripts/launch_train_multiprocessing.py

# 2. Clean up any stale Webots processes before restarting
pkill -9 -f webots

# 3. Inspect live Webots & supervisor logs if a worker hangs on pipe initialization
tail -n 30 /tmp/autotech/resmemo/webots_0.log /tmp/autotech/resmemo/vehicle_0_0.log