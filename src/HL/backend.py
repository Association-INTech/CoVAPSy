from flask import Flask, jsonify, Response
from flask_cors import CORS
import time
import logging as log
import numpy as np
import smbus as smbus  #type: ignore #ignore the module could not be resolved error because it is a linux only module
from Camera import Camera
import struct

# --------------------------------------------

# Initialise l'application Flask
app = Flask(__name__)
CORS(app)

# ---------------------------
# Creer une instance SMBus
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08

log.basicConfig(level=log.INFO) # Mettre log.DEBUG pour plus de détails
log.info("Initialisation de la caméra...")
cam = Camera()
log.info("Caméra initialisée.")
log.info("Démarrage du thread de capture...")
cam.start()
log.info("Thread de capture démarré.")

def lire_donnees_arduino():
    """
    Lit huit octets de l'Arduino et les convertit en 2 float.
    On veut la vitesse reel et la vitesse cible
    """
    try:
        # Demande 4 octets à l'esclave.
        # read_i2c_block_data(adresse, commande, nombre_octets)
        # La "commande" (le 0) n'est pas utilisée par notre Arduino,
        # mais le protocole l'exige.
        data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, 8)

        # Les données arrivent sous forme de liste :
        # data = [highByte1, lowByte1, highByte2, lowByte2]

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

def gen_frames():
    """Générateur qui lit le flux depuis l'objet Caméra."""
    log.info("Démarrage du flux vidéo.")
    while True:
        # Limite le framerate pour ne pas saturer le réseau
        time.sleep(0.05)  # ~20 images/seconde

        frame_bytes = None

        # Lit l'image la plus récente de manière thread-safe
        with cam.streaming_lock:
            frame_bytes = cam.streaming_frame

        if frame_bytes:
            # Envoie l'image au navigateur
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Définit la "route" principale de l'API
@app.route('/data')
def get_data():
    """
    Cette route exécute votre analyse d'image
    et renvoie les résultats.
    """
    vitesse_reelle,vitesse_cible = lire_donnees_arduino()
    donnees = {'vitesse_reelles':vitesse_reelle,'vitesse_cible':vitesse_cible}
    try:
        matrix = cam.camera_matrix()

        # Renvoyer des données utiles au format JSON
        if matrix is not None:
            data = {
                'matrix_sum': int(np.sum(matrix)),
                'matrix_mean': float(np.mean(matrix)),
                'total_red': int(np.count_nonzero(matrix == -1)),
                'total_green': int(np.count_nonzero(matrix == 1)),
            }
        else:
            data = {'error': 'Matrix could not be calculated'}

        return jsonify(data|donnees)
    except Exception as e:
        log.error(f"Erreur dans /data: {e}")
        return jsonify({'error': str(e)}), 500


@app.route('/video_feed')
def video_feed():
    """La route qui sert le flux vidéo."""
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# Point d'entrée pour démarrer le serveur
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)