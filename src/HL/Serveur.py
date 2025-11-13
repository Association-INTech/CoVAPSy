print("lancement...?")
import time
import logging
import sys
# from systemd.journal import JournalHandler #il y a des soucis dessus pour linstant
import zerorpc
import struct
import logging as log

from Camera import Camera
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


print("lancement...")

class ApiVoiture(): # pylint: disable=too-few-public-methods
    """
        ça controlera tout
    """

    def __init__(self):
        self.vitesse = 0 #vitesse en metre par seconde
        self.direction = 0 # direction en degrés avec 0 le degré du centre
        log.basicConfig(level=log.INFO)  # Mettre log.DEBUG pour plus de détails
        log.info("Initialisation de la caméra...")
        self.cam = Camera()
        log.info("Caméra initialisée.")
        log.info("Démarrage du thread de capture...")
        self.cam.start()
        log.info("Thread de capture démarré.")
        self.vitesse_m_s = 0
        self.direction = 0

    def write_vitesse_direction(self,vitesse, direction):
        # Convert string to list of ASCII values
        data = struct.pack('<ff', float(vitesse), float(direction))
        bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))

    def read_data(self,length):
        # Read a block of data from the slave
        data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
        # Convert the byte data to a float
        if len(data) >= 4:
            float_value = struct.unpack('f', bytes(data[:4]))[0]
            return float_value
        else:
            raise ValueError("Not enough data received from I2C bus")

    def lire_donnees_arduino(self):
        """
        Lit huit octets de l'Arduino et les convertit en 2 float.
        On veut la vitesse reel et la vitesse cible
        """
        try:
            # Demande 8 octets à l'esclave.
            data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, 8)
            # Reconstituer les entiers
            vitesse_reel = struct.unpack('<f', bytearray(data[0:4]))[0]
            vitesse_cible = struct.unpack('<f', bytearray(data[4:8]))[0]

            return vitesse_reel, vitesse_cible

        except IOError as e:
            print(f"Erreur I2C : {e}")
            return None, None
        except Exception as e:
            print(f"Erreur inattendue : {e}")
            return None, None

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