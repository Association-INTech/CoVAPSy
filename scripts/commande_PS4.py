from pyPS4Controller.controller import Controller
import time
import os
from threading import Thread
from programme import Program
from src.HL.Autotech_constant import MAX_ANGLE
###################################################
#Intialisation du protocole zmq
##################################################

def envoie_donnee(Voiture): #si utilisation de la voiture directement
    print("lancement de l'i2c")
    import smbus
    import struct
    from src.HL.Autotech_constant import SLAVE_ADDRESS

    bus = smbus.SMBus(1)
    while True:
            try :
                data = struct.pack('<ff', float(round(Voiture.vitesse_mms)), float(round(Voiture.direction)))
                bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                #time.sleep(0.00005)
            except Exception as e:
                print("i2c mort" + str(e))
                time.sleep(1)


#paramètres de la fonction vitesse_m_s, à étalonner
vitesse_max_m_s_soft = 2 #vitesse maximale que l'on souhaite atteindre en métre par seconde
vitesse_min_m_s_soft = -2 #vitesse arriere que l'on souhaite atteindre en métre

MAX_LEFT = -32767 + 3000   # deadzone 3000

# fonction naturel map de arduino pour plus de lisibilité
def map_range(x, in_min,in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class PS4ControllerProgram(Program):

    def __init__(self):
        super().__init__()
        self.name = "PS4 Controller"
        self.running = False
        self.controls_car = True

        #initialisation
        self.controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.controller.stop = True


    def start(self):
        self.running = True
        self.controller.stop = False
        self.thread = Thread(
            target=self.controller.listen, kwargs=dict(timeout=60),
            daemon=True
        )
        self.thread.start()

    def kill(self):
        self.controller.stop = True
        self.running = False

    @property
    def vitesse_d(self):
        return self.controller.vitesse_mms
    
    @property
    def direction(self):
        return self.controller.direction

class MyController(Controller):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.vitesse_mms = 0 # vitesse initiale en métre par milliseconde
        self.direction = 0 # angle initiale des roues en degrés
        self.filtered = 0
        self.alpha = 0.3
        self.running = 0

    def stable_direction(self,value):

        # Deadzone
        if value < MAX_LEFT:
            target = -MAX_ANGLE
        else:
            target = map_range(value, -32767, 0, -MAX_ANGLE, 0)

        # Low-pass filtering
        self.filtered = self.filtered * (1 - self.alpha) + target * self.alpha
        return self.filtered

        
    def on_R2_press(self,value):
        vit = map_range(value,-32252,32767,0,vitesse_max_m_s_soft*1000)
        if (vit < 0):
            self.vitesse_mms = 0
        else:
            self.vitesse_mms = vit
    def on_R2_release(self): # arrete la voiture lorsque L2 est arrété d'étre préssé. 
        self.vitesse_mms = 0
    
    
 
    def on_L3_x_at_rest(self):
        self.direction = 0
        
    def on_R1_press(self): #arret d'urgence
        self.vitesse_mms = 0
        
    def on_R1_release(self):
        self.vitesse_mms = 0
    


    def on_L3_right(self,value):
        # print("x_r :", value, "degré : ",map_range(value,-32767, 32767, 60, 120))
        dir = map_range(value, 0, 32767, 0, MAX_ANGLE)
        self.direction = dir

    def on_L3_left(self,value):
        #print("x_r :", value, "degré : ",map_range(value,-32767, 0, -MAX_ANGLE, 0 ))
        dir = self.stable_direction(value)
        self.direction = dir


    def on_L2_press(self, value):
        #print("x_r :", value, "degré : ",map_range(value,-32767, 32767, 60, 120))
        vit = map_range(value,-32252,32767,0,vitesse_min_m_s_soft*1000)
        if (vit > 0):
            self.vitesse_mms = 0
        else:
            self.vitesse_mms = vit
    
    def on_L2_release(self): #arrete la voiture lorsque L2 est arrété d'étre préssé. 
        self.vitesse_mms = 0
    
    def on_L3_up(self,value):
        pass
    def on_L3_down(self,value):
        pass
    def on_L3_y_at_rest(self):
        pass

if __name__ == "__main__":
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    try:
        Thread(target = envoie_donnee,args=(controller,), daemon=True).start()
        controller.listen(timeout=60)

    except KeyboardInterrupt:
        print("Arrêt du programme")
        controller.stop()
        exit(0)
