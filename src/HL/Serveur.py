import time
import sys
# from systemd.journal import JournalHandler #il y a des soucis dessus pour linstant
import zerorpc
import struct
import logging as log

# from Camera import Camera
import smbus #type: ignore

# uv pip install systemd-python
# uv pip install picamera2
# sudo apt install python3-libcamera
"""
Lancer un serveur qui tourne h24 qui gere l'intercommunication des processus (qui sont externe a la Pi)
Cf envoie vitesse a l'arduino, communication avec l'ecran , avec les boutons ,avec le lidar et la camera 
Tout doit passer par cette classe qui tourne s
"""

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08


class ApiVoiture(): # pylint: disable=too-few-public-methods
    """
        ça controlera tout
    """

    def __init__(self):
        self.vitesse_r = 0 # vitesse en metre par seconde réel
        self.vitesse_d = 0 # vitesse demander en metre par seconde
        self.direction = 0 # direction en degrés avec 0 le degré du centre
        self.voltage_lipo = 0
        self.voltage_nimh = 0
        log.basicConfig(level=log.INFO)  # Mettre log.DEBUG pour plus de détails
        log.info("Initialisation de la caméra...")
        # self.cam = Camera()
        log.info("Caméra initialisée.")
        log.info("Démarrage du thread de capture...")
        self.cam.start()
        log.info("Thread de capture démarré.")
        self.vitesse_m_s = 0
        self.direction = 0

    def write_vitesse_direction(self,vitesse, direction):

        self.vitesse_d = vitesse #on enregistre la vitesse demandé
        self.direction = direction # on enregistre la direction voulue

        data = struct.pack('<ff', float(vitesse), float(direction))
        bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))

    def read_data(self,num_floats=3):


        # Each float is 4 bytes
        length = num_floats * 4
        # Read a block of data from the slave
        data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
        # Convert the byte data to floats
        if len(data) >= length:
            float_values = struct.unpack('f' * num_floats, bytes(data[:length]))
            return list(float_values)

        else:
            raise ValueError("Not enough data received from I2C bus")


    def gen_frames(self):
        """Générateur qui lit le flux depuis l'objet Caméra."""
        log.info("Démarrage du flux vidéo.")
        while True:
            # Limite le framerate pour ne pas saturer le réseau
            time.sleep(0.05)  # ~20 images/seconde

            frame_bytes = None

            # Lit l'image la plus récente de manière thread-safe
            with self.cam.streaming_lock:
                frame_bytes = self.cam.streaming_frame

            if frame_bytes:
                # Envoie l'image au navigateur
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

if __name__ == '__main__':

    try:
        api = ApiVoiture()
        s = zerorpc.Server(api)
        s.bind("tcp://0.0.0.0:4242")
        print("Serveur ZERORPC lancé sur tcp://0.0.0.0:4242")
        s.run()
    except Exception as e:
        print("Erreur au démarrage :", e)