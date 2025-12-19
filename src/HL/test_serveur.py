import zmq
import base64
from io import BytesIO
from PIL import Image
import time
import numpy as np
# on envoie les données au serveur
PI_IP = "192.168.1.10"   # mets la bonne IP

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.connect(f"tcp://{PI_IP}:6001")
def get_camera_frame():
    socket.send_json({"cmd": "cam"})
    reply = socket.recv_json()

    if reply["cam"] is None:
        print("Image not ready")
        return None

    jpg_bytes = base64.b64decode(reply["cam"])   # <-- aligné ici
    img = Image.open(BytesIO(jpg_bytes))
    return img

"""
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://127.0.0.1:5557")

def recoit_donnee():
    socket.send_json({"cmd": "info"})
    resp = socket.recv_json()
    print(resp)

rDistance = None
xTheta = None

def startPlotter(self, autorange=False):
        def toCartesian(xTheta, xR):
            X = np.cos(xTheta) * xR
            Y = np.sin(xTheta) * xR
            return X,Y

        plt.show()
        fig = plt.figure()
        axc = plt.subplot(121)
        axp = plt.subplot(122, projection='polar')
        # axp.set_thetamax(deg2theta(45))
        # axp.set_thetamax(deg2theta(270 + 45))
        axp.grid(True)
        log.info('Plotter started, press any key to exit')

        log.debug(f'{xTheta}, {rDistance}')
        while True:
            X, Y = toCartesian(xTheta, rDistance)

            axp.clear()
            axc.clear()

            axp.plot(xTheta, rDistance)

            axc.plot(X, Y)

            if not autorange:
                axp.set_rmax(8000)
                axc.set_xlim(-5000, 5000)
                axc.set_ylim(-5000, 5000)

            plt.pause(1e-17)

            if plt.waitforbuttonpress(timeout=0.02):
                os._exit(0)

"""
import cv2

cap = cv2.VideoCapture("tcp://192.168.1.10:6002")


if __name__ == "__main__":
    while(True):
        ret, frame = cap.read()
        if not ret:
            print("No frame…")
            continue

        cv2.imshow("RPI H264 60fps", frame)
        if cv2.waitKey(1) == 27:
            break

        # Request data from the slave