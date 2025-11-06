from pyPS4Controller.controller import Controller
import time
from threading import Thread

#Pour le protocole I2C de communication entre la rasberie Pi et l'arduino
import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import numpy as np
import struct

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

direction_d = 90 # angle initiale des roues en degrés
vitesse_m = 0   # vitesse initiale en métre par milliseconde

#paramètres de la fonction vitesse_m_s, à étalonner
vitesse_max_m_s_hard = 8 #vitesse que peut atteindre la voiture en métre
vitesse_max_m_s_soft = 2 #vitesse maximale que l'on souhaite atteindre en métre par seconde
vitesse_min_m_s_soft = -2 #vitesse arriere que l'on souhaite atteindre en métre

angle_degre_max = +18 #vers la gauche



# fonction naturel map de arduino pour plus de lisibilité
def map_range(x, in_min,in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min



def set_direction_degre(angle_degre) :
    global direction_d
    direction_d = angle_degre
    print("angle_degré: ",direction_d,"vitesse: ",vitesse_m)
    
def set_vitesse_m_ms(vitesse_m_ms):
    global vitesse_m
    vitesse_m = vitesse_m_ms
    print("angle_degré: ",direction_d,"vitesse: ",vitesse_m)
        
def recule(): #actuellement ne sert a rien car on peux juste envoyer une vitesse négative 
    global vitesse_m
    vitesse_m = -2000


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
    def on_R2_press(self,value):
        vit = map_range(value,-32252,32767,0,vitesse_max_m_s_soft*1000)
        if (vit < 0):
            set_vitesse_m_ms(0)
        else:
            set_vitesse_m_ms(vit)
        
 
    def on_L3_x_at_rest(self):
        set_direction_degre(0)
        
    def on_R1_press(self): #arret d'urgence
        set_vitesse_m_ms(0)
        
    def on_R1_release(self):
        set_vitesse_m_ms(0)
    
    def on_L3_right(self,value):
        print("x_r :", value, "degré : ",map_range(value,-32767, 32767, 60, 120))
        dir = map_range(value, 0, 32767, 0, angle_degre_max)
        set_direction_degre(dir)

    def on_L3_left(self,value):
        dir = map_range(value,-32767, 0, -angle_degre_max, 0 )
        print("x_l :", value)
        set_direction_degre(dir)


    def on_L2_press(self, value):
        vit = map_range(value,-32252,32767,0,vitesse_min_m_s_soft*1000)
        if (vit > 0):
            set_vitesse_m_ms(0)
        else:
            set_vitesse_m_ms(vit)

#envoie de la direction et de l'angle toute les millisecondes
def envoie_direction_degre():
    while True :
        write_vitesse_direction(int(vitesse_m), int(direction_d))
        time.sleep(0.001)


# boucle principal
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
try:
    Thread(target = envoie_direction_degre, daemon=True).start()
    controller.listen(timeout=60)

except KeyboardInterrupt:
    print("Arrêt du programme")
    controller.stop()
    exit(0)


