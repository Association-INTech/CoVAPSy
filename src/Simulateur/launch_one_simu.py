import os
import sys

from typing import *
import numpy as np
import onnxruntime as ort
import gymnasium as gym


from onnx_utils import run_onnx_model
from config import *
from TemporalResNetExtractor import TemporalResNetExtractor
from CNN1DResNetExtractor import CNN1DResNetExtractor
# -------------------------------------------------------------------------

def log(s: str):
    if B_DEBUG:
        print(s, file=open("/tmp/autotech/logs", "a"))


ONNX_MODEL_PATH = "model.onnx"

# --- Initialisation du moteur d'inférence ONNX Runtime (ORT) ---
def init_onnx_runtime_session(onnx_path: str) -> ort.InferenceSession:
    if not os.path.exists(onnx_path):
        raise FileNotFoundError(f"Le fichier ONNX est introuvable à : {onnx_path}. Veuillez l'exporter d'abord.")

    # Crée la session d'inférence
    return ort.InferenceSession(onnx_path) #On peut modifier le providers afin de mettre une CUDA


class WebotsSimulationGymEnvironment(gym.Env):
    """
    One environment for each vehicle

    n: index of the vehicle
    supervisor: the supervisor of the simulation
    """

    def __init__(self, simulation_rank: int):
        super().__init__()
        self.simulation_rank = simulation_rank

        # this is only true if lidar_horizontal_resolution = camera_horizontal_resolution
        box_min = np.zeros([2, context_size, lidar_horizontal_resolution], dtype=np.float32)
        box_max = np.ones([2, context_size, lidar_horizontal_resolution], dtype=np.float32) * 30

        self.observation_space = gym.spaces.Box(box_min, box_max, dtype=np.float32)
        self.action_space = gym.spaces.MultiDiscrete([n_actions_steering, n_actions_speed])

        if not os.path.exists("/tmp/autotech"):
            os.mkdir("/tmp/autotech")

        log(f"SERVER{simulation_rank} : {simulation_rank=}")

        os.mkfifo(f"/tmp/autotech/{simulation_rank}toserver.pipe")
        os.mkfifo(f"/tmp/autotech/serverto{simulation_rank}.pipe")

        #  --mode=fast --minimize --no-rendering --batch --stdout
        os.system(f"""
            webots {__file__.rsplit('/', 1)[0]}/worlds/piste{simulation_rank % n_map}.wbt --mode=fast --minimize --no-rendering --batch --stdout &
            echo $! {simulation_rank} >>/tmp/autotech/simulationranks
        """)
        log(f"SERVER{simulation_rank} : {simulation_rank}toserver.pipe")
        self.fifo_r = open(f"/tmp/autotech/{simulation_rank}toserver.pipe", "rb")
        log(f"SERVER{simulation_rank} : serverto{simulation_rank}.pipe")
        self.fifo_w = open(f"/tmp/autotech/serverto{simulation_rank}.pipe", "wb")
        log(f"SERVER{simulation_rank} : fifo opened :D and init done")
        log("-------------------------------------------------------------------")

    def reset(self, seed=0):
        # basically useless function
        # lidar data
        # this is true for lidar_horizontal_resolution = camera_horizontal_resolution
        self.context = obs = np.zeros([2, context_size, lidar_horizontal_resolution], dtype=np.float32)
        info = {}
        return obs, info

    def step(self, action):
        log(f"SERVER{self.simulation_rank} : sending {action=}")
        self.fifo_w.write(action.astype(np.int64).tobytes())
        self.fifo_w.flush()

        # communication with the supervisor
        cur_state   = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize * (n_sensors + lidar_horizontal_resolution + camera_horizontal_resolution)), dtype=np.float32)
        log(f"SERVER{self.simulation_rank} : received {cur_state=}")
        reward      = np.frombuffer(self.fifo_r.read(np.dtype(np.float32).itemsize), dtype=np.float32)[0] # scalar
        log(f"SERVER{self.simulation_rank} : received {reward=}")
        done        = np.frombuffer(self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool)[0] # scalar
        log(f"SERVER{self.simulation_rank} : received {done=}")
        truncated   = np.frombuffer(self.fifo_r.read(np.dtype(np.bool).itemsize), dtype=np.bool)[0] # scalar
        log(f"SERVER{self.simulation_rank} : received {truncated=}")
        info        = {}

        cur_state = np.nan_to_num(cur_state[n_sensors:], nan=0., posinf=30.)

        lidar_obs = cur_state[:lidar_horizontal_resolution]
        camera_obs = cur_state[lidar_horizontal_resolution:]

        # apply dropout to the camera
        # p = 0.5
        # camera_obs *= np.random.binomial(1, 1-p, camera_obs.shape) # random values in {0, 1}

        self.context = obs = np.concatenate([
            self.context[:, 1:],
            [lidar_obs[None], camera_obs[None]]
        ], axis=1)
        # check if the context is correct
        # if self.simulation_rank == 0:
        #     print(f"{(obs[0] == 0).mean():.3f} {(obs[1] == 0).mean():.3f}")
        return obs, reward, done, truncated, info

    def close(self):
        print("Fermeture de l'environnement...")
        if hasattr(self, 'fifo_r') and self.fifo_r:
            self.fifo_r.close()
        if hasattr(self, 'fifo_w') and self.fifo_w:
            self.fifo_w.close()

        # Nettoyage des fichiers pipes
        if hasattr(self, 'pipe_name_read') and os.path.exists(self.pipe_name_read):
            os.unlink(self.pipe_name_read)
        if hasattr(self, 'pipe_name_write') and os.path.exists(self.pipe_name_write):
            os.unlink(self.pipe_name_write)


if __name__ == "__main__":
    if not os.path.exists("/tmp/autotech/"):
        os.mkdir("/tmp/autotech/")

    os.system('if [ -n "$(ls /tmp/autotech)" ]; then rm /tmp/autotech/*; fi')
    if B_DEBUG:
        print("Webots started", file=open("/tmp/autotech/logs", "w"))


    # 2. Initialisation de la session ONNX Runtime
    try:
        ort_session = init_onnx_runtime_session(ONNX_MODEL_PATH)
        input_name = ort_session.get_inputs()[0].name
        output_name = ort_session.get_outputs()[0].name
        print(f"Modèle ONNX chargé depuis {ONNX_MODEL_PATH}")
        print(f"Input Name: {input_name}, Output Name: {output_name}")
    except FileNotFoundError as e:
        print(f"ERREUR : {e}")
        print(
            "Veuillez vous assurer que vous avez exécuté une fois le script d'entraînement pour exporter 'model.onnx'.")
        sys.exit(1)

    env = WebotsSimulationGymEnvironment(0)
    obs,_ = env.reset()

    print("Début de la simulation en mode inférence...")

    max_steps = 5000
    step_count = 0

    while True:
        action = run_onnx_model(ort_session,obs[None])

        # 4. Exécuter l'action dans l'environnement
        obs, reward, done,truncated, info = env.step(action)

        # Gestion des fins d'épisodes
        if done:
            print(f"Épisode(s) terminé(s) après {step_count} étapes.")
            obs,_ = env.reset()



    # Fermeture propre (très important pour les processus parallèles SubprocVecEnv)
    envs.close()
    print("Simulation terminée. Environnements fermés.")