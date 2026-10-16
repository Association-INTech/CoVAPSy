import configparser 
import json
import sys

# Lecture du fichier de configuration

config = configparser.ConfigParser()
config.read('config.ini')

# Modification du fichier de configuration :



# Conversion du fichier de configuration en dictionnaire puis en JSON
config_dict = {section: dict(config[section]) for section in config.section()}

# Affichage du JSON pour que JS puisse lire le fichier
print(json.dumps(config_dict))
