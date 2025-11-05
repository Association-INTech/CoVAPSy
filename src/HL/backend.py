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

c = zerorpc.Client()
c.connect(f"tcp://{IP_DU_RASPBERRY_PI}:4242")
# ---------------------------
# Creer une instance SMBus
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08


# Définit la "route" principale de l'API
@app.route('/data')
def get_data():
    """
    Cette route exécute votre analyse d'image
    et renvoie les résultats.
    """
    vitesse_reelle = c.read_data(3)[2]
    donnees = {'vitesse_reelles':vitesse_reelle}
    try:
        matrix = c.cam.camera_matrix()

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
    return Response(c.gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# Point d'entrée pour démarrer le serveur
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)