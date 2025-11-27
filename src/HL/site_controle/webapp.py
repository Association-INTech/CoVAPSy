from flask import Flask, jsonify, render_template, Response, request
from flask_cors import CORS
import zmq
from Camera import Camera

app = Flask(__name__, template_folder=".")  # Cherche les html dans le dossier courant
CORS(app)

# --- Configuration ZMQ (Le Client) ---
context = zmq.Context()
# On se connecte au port 5557 défini dans Serveur_mq.py
mq_socket = context.socket(zmq.REQ)
mq_socket.connect("tcp://127.0.0.1:5557")


@app.route('/')
def index():
    # Assurez-vous que controle.html est dans le même dossier
    return render_template('controle.html')


@app.route('/data')
def get_data():
    """ Demande les infos à Serveur_mq.py """
    try:
        # 1. Envoi de la demande
        mq_socket.send_json({"cmd": "info"})
        # 2. Réception de la réponse (bloquant)
        data = mq_socket.recv_json()

        # On restructure pour correspondre à ce que le HTML attend
        formatted_data = {
            "batterie": {
                "lipo": data.get("voltage_lipo", 0),
                "nimh": data.get("voltage_nimh", 0)
            },
            "robot": {
                "vitesse": data.get("vitesse_reelle", 0),
                "programme": data.get("programme_actuel", "Inconnu"),
                "lidar": data.get("lidar_data", [])
            }
        }
        return jsonify(formatted_data)
    except Exception as e:
        # Si ZMQ plante (timeout), on réinitialise le socket
        print(f"Erreur ZMQ: {e}")
        global mq_socket
        mq_socket.close()
        mq_socket = context.socket(zmq.REQ)
        mq_socket.connect("tcp://127.0.0.1:5557")
        return jsonify({"error": "Pas de connexion au robot"}), 500


@app.route('/set_program/<int:prog_id>', methods=['POST'])
def set_program(prog_id):
    """ Envoie l'ordre de changer de programme """
    try:
        mq_socket.send_json({"cmd": "change_program", "id": prog_id})
        response = mq_socket.recv_json()
        return jsonify(response)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/video_feed')
def video_feed():
    """La route qui sert le flux vidéo."""
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# --- Route Vidéo (Si vous avez intégré la classe Camera ici ou via un import) ---
# Voir la réponse précédente pour l'implémentation de gen_frames()
# @app.route('/video_feed')
# def video_feed():
#     return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)