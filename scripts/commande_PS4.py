from pyPS4Controller.controller import Controller
import time
from threading import Thread

###################################################
#Intialisation du protocole zmq
##################################################
import zmq
# on envoie les données au serveur
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://127.0.0.1:5555")

def envoie_donnee():
    global vitesse_m , direction_d
    while(True):
        socket.send_json({"cmd": "set_speed", "value": vitesse_m})
        resp = socket.recv_json()
        socket.send_json({"cmd": "set_direction", "value": direction_d})
        resp = socket.recv_json()
        time.sleep(0.02)

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
    def on_R2_release(self): # arrete la voiture lorsque L2 est arrété d'étre préssé. 
        set_vitesse_m_ms(0)
    
    
 
    def on_L3_x_at_rest(self):
        set_direction_degre(0)
        
    def on_R1_press(self): #arret d'urgence
        set_vitesse_m_ms(0)
        
    def on_R1_release(self):
        set_vitesse_m_ms(0)
    
    def on_L3_up(self,value):
        pass
    def on_L3_down(self,value):
        pass


    def on_L3_right(self,value):
        # print("x_r :", value, "degré : ",map_range(value,-32767, 32767, 60, 120))
        dir = map_range(value, 0, 32767, 0, angle_degre_max)
        set_direction_degre(dir)

    def on_L3_left(self,value):
        print("x_r :", value, "degré : ",map_range(value,-32767, 0, -angle_degre_max, 0 ))
        dir = map_range(value,-32767, 0, -angle_degre_max, 0 )
        set_direction_degre(dir)


    def on_L2_press(self, value):
        print("x_r :", value, "degré : ",map_range(value,-32767, 32767, 60, 120))
        vit = map_range(value,-32252,32767,0,vitesse_min_m_s_soft*1000)
        if (vit > 0):
            set_vitesse_m_ms(0)
        else:
            set_vitesse_m_ms(vit)
    
    def on_L2_release(self): #arrete la voiture lorsque L2 est arrété d'étre préssé. 
        set_vitesse_m_ms(0)


# boucle principal
controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
try:
    Thread(target = envoie_donnee, daemon=True).start()
    controller.listen(timeout=60)

except KeyboardInterrupt:
    print("Arrêt du programme")
    controller.stop()
    exit(0)


