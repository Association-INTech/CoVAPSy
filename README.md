# CoVAPSy - AutoTech Project

AutoTech is a project from the INTech robotics club (Telecom SudParis and IMT Business School). Our goal is to design and build an autonomous vehicle for [CoVAPSy](https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/) (Course de Voitures Autonomes de Paris-Saclay) competition.

This repository contains the source code for our autonomous vehicle for the 2025 and 2026 competitions.

## Our Approach

We chose to use a full Reinforcement Learning approach. We train an agent to drive the vehicle in a simulation of the race based directly on sensor inputs.

We train the AI model in parallel remote environments through nammed pipes connections between the main python script and the multiple webots instances.

Inside the simulations, each vehicle has access to data comming from a LiDAR and a camera.

- Simulator: [Webots](https://cyberbotics.com/)
- AI Training Library: [Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/) with a [Pytorch](https://pytorch.org/) backend
- AI Inference Engine: [ONNX](https://onnx.ai/)

## Installation

We use uv for Python environment management. So if it's not already installed go check the [official installation guide](https://docs.astral.sh/uv/getting-started/installation/)

Then, just `uv sync` to create the virtual environment and get all the dependencies.
```bash
# dependencies for AI training
uv sync --extra simu

# dependencies for AI inference on the Raspberry PI 5
uv sync --extra rpi
```

## Training usage

Navigate to the simulator directory.
```bash
cd src/Simulateur
```

Run the multi-process training script.
```bash
uv run launch_train_multiprocessing.py
```
This will launch the Webots instances and begin the SB3 PPO training loop. All the checkpoints will be in the `checkpoints` directory. At every checkpoint, a compiled ONNX model will be stored as `model.onnx`.

To change the parameters of the simulation, just modify the `config.py` file.

## Inference usage

(WIP)

# Wiki (Documentation)

For detailed information on architecture, hardware specifics and technical choices, please refer to the [INTech wiki](https://wiki.intech-robotics.fr).

(Note: The Wiki is currently private and reserved to INTech members)


# some dependencies needed 
libcap-dev
python3-libcamera

# License

This project is distributed under the MIT License. See the Licence file for details.
