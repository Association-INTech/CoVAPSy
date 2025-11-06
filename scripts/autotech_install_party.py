from Lidar import Lidar
import time
from threading import Thread

#Pour le protocole I2C de communication entre la rasberie Pi et l'arduino
import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import numpy as np
import struct

SOCKET_ADRESS = {
    "IP": '192.168.0.10',
    "PORT": 10940
}


"""Initialize the Lidar sensor."""
try:
    lidar = Lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
    lidar.stop()
    lidar.startContinuous(0, 1080)
    print("Lidar initialized successfully")
except Exception as e:
    raise




###################################################
#Intialisation du protocole I2C
##################################################

# Create an SMBus instance
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08

def write_vitesse_direction(vitesse,direction):
    # Convert string to list of ASCII values
    data = struct.pack('<ff', float(vitesse), float(direction))
    bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))

###################################################
#Intialisation des moteurs
##################################################

direction_d = 0 # angle initiale des roues en degrés
vitesse_m = 0   # vitesse initiale en métre par milliseconde

#paramètres de la fonction vitesse_m_s, à étalonner
vitesse_max_m_s_hard = 8 #vitesse que peut atteindre la voiture en métre
vitesse_max_m_s_soft = 2 #vitesse maximale que l'on souhaite atteindre en métre par seconde
vitesse_min_m_s_soft = -2 #vitesse arriere que l'on souhaite atteindre en métre

angle_degre_max = +18 #vers la gauche



def set_vitesse_m_s(vitesse_m_s):
    global vitesse_m
    vitesse_m = vitesse_m_s
        
def recule():
    global vitesse_m
    vitesse_m = -1
    

def set_direction_degre(angle_degre) :
    global direction_d
    direction_d = angle_degre
    
    
#connexion et démarrage du lidar
lidar = RPLidar("/dev/ttyUSB0",baudrate=256000)
lidar.connect()
print (lidar.get_info())
lidar.start_motor()
time.sleep(1)

tableau_lidar_mm = [0]*360 #création d'un tableau de 360 zéros


def envoie_direction_degre():
    while True :
        write_vitesse_direction(int(vitesse_m), int(direction_d))
        time.sleep(0.001)

Thread(target = envoie_direction_degre, daemon=True).start()

try : 
    lidar.startContinuous(0, 1080)  #scan[i][2] : distance    
    #############################################
    ## Code de conduite (issu du simulateur ou non)
    #############################################
    lidar_data = (lidar.rDistance[:1080]/1000)

    #l'angle de la direction est la différence entre les mesures  
    #des rayons du lidar à -60 et +60°  
    angle_degre = 0.02*(lidar_data[3]-lidar_data[1077])
    set_direction_degre(angle_degre)
    vitesse_m_s = 0.5
    set_vitesse_m_s(vitesse_m_s)    
    ##############################################
except KeyboardInterrupt: #récupération du CTRL+C
    print("fin des acquisitions")

#arrêt et déconnexion du lidar et des moteurs
lidar.stop_motor()
lidar.stop()
time.sleep(1)
lidar.disconnect()