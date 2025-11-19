import zmq
import time
# on envoie les données au serveur
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


if __name__ == "__main__":
    while(True):
        recoit_donnee()
        time.sleep(0.1)  # Wait for the slave to process the data+ù

        # Request data from the slave