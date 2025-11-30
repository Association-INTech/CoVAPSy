import os
import sys

from typing import *
import numpy as np
import onnxruntime as ort

simu_path = __file__.rsplit('/', 2)[0] + '/src/Simulateur'
if simu_path not in sys.path:
    sys.path.insert(0, simu_path)

from onnx_utils import run_onnx_model
from config import *
from WebotsSimulationGymEnvironment import WebotsSimulationGymEnvironment
from TemporalResNetExtractor import TemporalResNetExtractor
from CNN1DResNetExtractor import CNN1DResNetExtractor
# -------------------------------------------------------------------------

def log(s: str):
    # Conservez votre fonction de log
    if B_DEBUG:
        print(s, file=open("/tmp/autotech/logs", "a"))

# --- Chemin vers le fichier ONNX ---

ONNX_MODEL_PATH = "model.onnx"

# --- Initialisation du moteur d'inférence ONNX Runtime (ORT) ---
def init_onnx_runtime_session(onnx_path: str) -> ort.InferenceSession:
    if not os.path.exists(onnx_path):
        raise FileNotFoundError(f"Le fichier ONNX est introuvable à : {onnx_path}. Veuillez l'exporter d'abord.")

    # Crée la session d'inférence
    return ort.InferenceSession(onnx_path) #On peut modifier le providers afin de mettre une CUDA


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

    # 3. Boucle d'inférence (Test)
    env = WebotsSimulationGymEnvironment(0,0)
    obs = env.reset()
    print("Début de la simulation en mode inférence...")

    max_steps = 5000
    step_count = 0

    while True:

        action = run_onnx_model(ort_session,obs)

        # 4. Exécuter l'action dans l'environnement
        obs, reward, done, info = env.step(action)

        # Note: L'environnement Webots gère généralement son propre affichage
        # env.render() # Décommenter si votre env supporte le rendu externe

        # Gestion des fins d'épisodes
        if done:
            print(f"Épisode(s) terminé(s) après {step_count} étapes.")
            obs = env.reset()



    # Fermeture propre (très important pour les processus parallèles SubprocVecEnv)
    envs.close()
    print("Simulation terminée. Environnements fermés.")