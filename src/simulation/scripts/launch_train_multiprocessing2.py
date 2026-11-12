import subprocess
import sys
import os
import signal

while True:
    print("Lancement de ltm")

    processus = subprocess.Popen(
        [sys.executable, "-u", "launch_train_multiprocessing.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True
    )

    restart = False

    for ligne in processus.stdout:
        print(ligne, end="")

        if ligne.strip() == "RESTART":
            print("Interruption de ltm (Ctrl+C)...")

            restart = True

            # Envoie Ctrl+C à B et à ses sous-processus
            os.killpg(processus.pid, signal.SIGINT)
            break

    if restart:
        try:
            # Attend la fermeture de B
            processus.wait(timeout=10)

        except subprocess.TimeoutExpired:
            print("Fermeture forcée...")
            os.killpg(processus.pid, signal.SIGKILL)
            processus.wait()
    else:
        processus.wait()

    processus.stdout.close()

    if restart or processus.returncode == 42:
        print("Redémarrage...\n")
    else:
        print("Arrêt définitif")
        break