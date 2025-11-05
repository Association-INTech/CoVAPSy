import time
import logging
import sys
import zerorpc
import struct
import logging as log

from Camera import Camera
import smbus #type: ignore
"""
Lancer un serveur qui tourne h24 qui gere l'intercommunication des processus (qui sont externe a la Pi)
Cf envoie vitesse a l'arduino, communication avec l'ecran , avec les boutons ,avec le lidar et la camera 
Tout doit passer par cette classe qui tourne s
"""

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08


vitesse = 200 # en millimetre par seconde
direction = 100 # en degré


class ApiVoiture(): # pylint: disable=too-few-public-methods
    """
        ça controlera tout
    """

    def __init__(self):
        log.basicConfig(level=log.INFO)  # Mettre log.DEBUG pour plus de détails
        log.info("Initialisation de la caméra...")
        self.cam = Camera()
        log.info("Caméra initialisée.")
        log.info("Démarrage du thread de capture...")
        self.cam.start()
        log.info("Thread de capture démarré.")

    def write_vitesse_direction(self,vitesse, direction):
        # Convert string to list of ASCII values
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

    s  = zerorpc.Server(ApiVoiture())
    s.bind("tcp://0.0.0.0:4242")
    s.run()