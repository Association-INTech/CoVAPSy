from flask import Flask, jsonify, Response
from flask_cors import CORS
import time
import logging as log
import numpy as np
import smbus as smbus  #type: ignore #ignore the module could not be resolved error because it is a linux only module
from Camera import Camera
import zerorpc
import struct

# --------------------------------------------

# Initialise l'application Flask
app = Flask(__name__)
CORS(app)

IP_DU_RASPBERRY_PI = "192.168.1.25"

c = c = zerorpc.Client()
c.connect("tcp://127.0.0.1:4242")
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