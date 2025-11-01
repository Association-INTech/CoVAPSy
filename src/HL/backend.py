from flask import Flask, jsonify
from flask_cors import CORS
import time
# --- NOUVELLES IMPORTATIONS POUR LES CAPTEURS ---

import smbus as smbus  #type: ignore #ignore the module could not be resolved error because it is a linux only module
# --------------------------------------------

# Initialise l'application Flask
app = Flask(__name__)
CORS(app)

# ---------------------------
# Create an SMBus instance
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08

def lire_donnees_arduino():
    """
    Lit 4 octets de l'Arduino et les convertit en 2 entiers.
    On veut la vitesse reel et la vitesse cible
    """
    try:
        # Demande 4 octets à l'esclave.
        # read_i2c_block_data(adresse, commande, nombre_octets)
        # La "commande" (le 0) n'est pas utilisée par notre Arduino,
        # mais le protocole l'exige.
        data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, 4)

        # Les données arrivent sous forme de liste :
        # data = [highByte1, lowByte1, highByte2, lowByte2]

        # Reconstituer les entiers
        valeur1 = (data[0] << 8) | data[1]
        valeur2 = (data[2] << 8) | data[3]

        return valeur1, valeur2

    except IOError as e:
        print(f"Erreur I2C : {e}")
        return None, None
    except Exception as e:
        print(f"Erreur inattendue : {e}")
        return None, None



# Définit la "route" principale de l'API
@app.route('/data')
def get_data():
    vitesse_reelle,vitesse_cible = lire_donnees_arduino()
    donnees = {'vitesse_reelles':vitesse_reelle,'vitesse_cible':vitesse_cible}
    return jsonify(donnees)


# Point d'entrée pour démarrer le serveur
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)