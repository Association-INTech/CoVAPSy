# you must run export PYTHONPATH=/home/intech/CoVAPSy/src/high_level before running this script

import signal
import time

import numpy as np

from src.actionneur_capteur.lidar import Lidar

IP = "192.168.0.10"
PORT = 10940
START = 0
END = 1080

running = True


def signal_handler(sig, frame):
    global running
    print("\nArrêt demandé (Ctrl+C).")
    running = False


def main():
    global running

    signal.signal(signal.SIGINT, signal_handler)

    print("Initialisation du lidar...")
    lidar = Lidar(IP, PORT, start_step=START)

    print("Démarrage scan continu...")
    lidar.start_continuous(START, END)

    while np.all(lidar.r_distance == 0):
        time.sleep(0.01)

    print("Scan reçu. Début enregistrement min...")
    print("Appuie sur Ctrl+C pour arrêter proprement.")

    min_distance = np.full(lidar.r_distance.shape, np.inf, dtype=float)

    while running:
        current = lidar.r_distance.astype(float)
        valid = current > 0

        min_distance[valid] = np.minimum(min_distance[valid], current[valid])

        time.sleep(0.01)

    print("Arrêt du lidar...")
    lidar.stop()

    result = min_distance
    np.save("src/programs/data/min_lidar.npy", result)

    print("Sauvegardé dans ../src/high_level/programs/data/min_lidar.npy")
    print("Shape:", result.shape)


if __name__ == "__main__":
    main()
