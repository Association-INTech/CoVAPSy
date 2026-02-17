import time
import numpy as np
from src.actionneur_capteur.lidar import Lidar
import signal
import sys

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
    lidar = Lidar(IP, PORT, startStep=START)

    print("Démarrage scan continu...")
    lidar.startContinuous(START, END)

    while np.all(lidar.rDistance == 0):
        time.sleep(0.01)

    print("Scan reçu. Début enregistrement min...")
    print("Appuie sur Ctrl+C pour arrêter proprement.")

    min_distance = np.full(lidar.rDistance.shape, np.inf, dtype=float)

    while running:
        current = lidar.rDistance.astype(float)
        valid = current > 0

        min_distance[valid] = np.minimum(
            min_distance[valid],
            current[valid]
        )

        time.sleep(0.01)

    print("Arrêt du lidar...")
    lidar.stop()

    result = np.vstack((lidar.xTheta, min_distance)).T
    np.save("../src/high_level/data/min_lidar.npy", result)

    print("Sauvegardé dans min_lidar.npy")
    print("Shape:", result.shape)


if __name__ == "__main__":
    main()