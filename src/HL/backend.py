from flask import Flask, jsonify, Response, render_template
from flask_cors import CORS
from programme import Program

# --------------------------------------------
class Backend(Program):
    """ ce programme permet de controler le site via le site web"""

    def __init__(self,server):
        super().__init__()
        self.name = "Backend"
        self.controls_car = False
        self.server = server
        self.running = False
        self.app = Flask(__name__, template_folder="/site_controle")                                                     # Cherche les html dans le dossier site controle
        CORS(self.app)

        self.app.add_url_rule('/', view_func=self.index)                                                            # equivalent à @self.app.route dans nos fonction
        self.app.add_url_rule('/data', view_func=self.get_data)
        self.app.add_url_rule('/video_feed', view_func=self.get_video)
        self.app.add_url_rule('/set_program/<int:prog_id>', view_func=self.set_program, methods=['POST'])

    def start(self):
        self.running = True
        self.app.run(host='0.0.0.0', port=5000) # lance le serveur

    def kill(self):
        self.running = False

    def index(self):
        return render_template('controle.html')

    def get_data(self):
        """ Demande les infos à Serveur """
        data = self.server.i2c_received()

        formatted_data = {
            "batterie": {
                "lipo": data.get("voltage_lipo", 0),
                "nimh": data.get("voltage_nimh", 0)
            },
            "robot": {
                "vitesse": data.get("vitesse_reelle", 0),
                "programme": data.get("programme_actuel", "Inconnu"),     # Cyril si tu peux envoyer le programme actuel dans le serv ça m'arrangerrai
                "lidar": self.server.lidar()             #je suis pas sur de ça
            }
        }
        return jsonify(formatted_data)

    def set_program(self,prog_id):
       self.server.start_process(prog_id)
       return jsonify({"status": "ok", "program_id": prog_id})

    def get_video(self):
        self.server.camera()         # pas sur non plus                                                                                    #
        return Response(self.server.camera(), mimetype='multipart/x-mixed-replace; boundary=frame')

#-------------------------------------------
