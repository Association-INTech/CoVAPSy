import os
import sys

from typing import *
import numpy as np
import onnxruntime as ort
import gymnasium as gym

from simulation.config import *
from utils import run_onnx_model

from extractors import (  # noqa: F401
    CNN1DResNetExtractor,
    TemporalResNetExtractor,
)

from simulation import VehicleEnv

# -------------------------------------------------------------------------

ONNX_MODEL_PATH = "/home/exo/Bureau/CoVAPSy/model.onnx"


# --- Launching of inference motor ONNX Runtime (ORT) ---
def init_onnx_runtime_session(onnx_path: str) -> ort.InferenceSession:
    if not os.path.exists(onnx_path):
        raise FileNotFoundError(f"Le fichier ONNX est introuvable à : {onnx_path}. Veuillez l'exporter d'abord.")
    return ort.InferenceSession(onnx_path)


if __name__ == "__main__":
    if not os.path.exists("/tmp/autotech/"):
        os.mkdir("/tmp/autotech/")

    os.system('if [ -n "$(ls /tmp/autotech)" ]; then rm /tmp/autotech/*; fi')

    # Starting of OnnxSession
    try:
        ort_session = init_onnx_runtime_session(ONNX_MODEL_PATH)
        input_name = ort_session.get_inputs()[0].name
        output_name = ort_session.get_outputs()[0].name
        print(f"Modèle ONNX chargé depuis {ONNX_MODEL_PATH}")
        print(f"Input Name: {input_name}, Output Name: {output_name}")
    except FileNotFoundError as e:
        print(f"ERREUR : {e}")
        sys.exit(1)

    env = VehicleEnv(0, 0)
    obs, _ = env.reset()

    print("Début de la simulation en mode inférence...")

    step_count = 0

    while True:
        # 1. On récupère les logits (probabilités) bruts de l'ONNX
        raw_action = run_onnx_model(ort_session, obs[None])
        logits = np.array(raw_action).flatten()

        # 2. On sépare le tableau en deux (Direction et Vitesse)
        # On utilise n_actions_steering et n_actions_speed venant de config.py
        steer_logits = logits[:n_actions_steering]
        speed_logits = logits[n_actions_steering:]

        # 3. L'IA choisit l'action qui a le score (logit) le plus élevé
        action_steer = np.argmax(steer_logits)
        action_speed = np.argmax(speed_logits)

        # 4. On crée le tableau final parfaitement formaté pour Webots (strictement 2 entiers)
        action = np.array([action_steer, action_speed], dtype=np.int64)

        # 5. Exécuter l'action dans l'environnement
        next_obs, reward, done, truncated, info = env.step(action)

        step_count += 1

        # Gestion des fins d'épisodes
        if done:
            print(f"Épisode(s) terminé(s) après {step_count} étapes.")
            step_count = 0

            fresh_frame = next_obs[:, -1:]
            obs, _ = env.reset()
            env.context[:, -1:] = fresh_frame
            obs = env.context
        else:
            obs = next_obs

    env.close()
    print("Simulation terminée. Environnements fermés.")