import subprocess
import sys

while True:
    print("launch training")

    resultat = subprocess.run([sys.executable, "launch_train_multiprocessing.py"])

    if resultat.returncode == 3:
        print("iteration over, restarting")
    else:
        print("stop")
        break