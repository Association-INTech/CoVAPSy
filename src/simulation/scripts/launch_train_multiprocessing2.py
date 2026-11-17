import subprocess
import sys
import time

while True:
    print("launch training")

    resultat = subprocess.run([sys.executable, "launch_train_multiprocessing.py"])
    subprocess.run(["pkill", "webots"])
    time.sleep(3)
    if resultat.returncode == 3:
        print("iteration over, restarting")
    else:
        print("stop")
        break