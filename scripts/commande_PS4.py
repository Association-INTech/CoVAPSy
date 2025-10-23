from pyPS4Controller.controller import Controller
import time
from masterI2C import write_vitesse_direction
from threading import Thread

###################################################
#Intialisation des moteurs
##################################################

direction_d = 90
vitesse_m = 0
#paramètres de la fonction vitesse_m_s, à étalonner
direction_prop = 1# -1 pour les variateurs inversés ou un petit rapport correspond à une marche avant
pwm_stop_prop = 7.37
point_mort_prop = 0.5
delta_pwm_max_prop = 1.1 #pwm à laquelle on atteint la vitesse maximale

vitesse_max_m_s_hard = 8 #vitesse que peut atteindre la voiture
vitesse_max_m_s_soft = 2 #vitesse maximale que l'on souhaite atteindre
vitesse_min_m_s_soft = -2 #vitesse arriere que l'on souhaite atteindre

direction = -1 #1 pour angle_pwm_min a gauche, -1 pour angle_pwm_min à droite
angle_pwm_min = 6.91  #min
angle_pwm_max = 10.7  #max
angle_pwm_centre= 8.805
angle_degre_max = +18 #vers la gauche
angle_degre=0

def envoie_direction_degre():
    while True :
        write_vitesse_direction(int(vitesse_m), int(direction_d))
        time.sleep(0.001)
#on recode map de arduino pour des soucis 

def map(x, in_min,in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def set_direction_degre(angle_degre) :
    global direction_d
    direction_d = angle_degre
    print("angle_degré: ",direction_d,"vitesse: ",vitesse_m)
    #write_vitesse_direction(int(vitesse_m), int(direction_d)) #take the angle in degrees
    
def set_vitesse_m_ms(vitesse_m_ms):
    global vitesse_m
    vitesse_m = vitesse_m_ms
    print("angle_degré: ",direction_d,"vitesse: ",vitesse_m)
    #write_vitesse_direction(int(vitesse_m), int(direction_d)) # Convert to millimeters per second
        
def recule():
    global vitesse_m
    vitesse_m = -2000
    #write_vitesse_direction(int(vitesse_m), int(direction_d)) # Convert to millimeters per second


a_prop = vitesse_max_m_s_soft/(65198)
a_dir=(angle_degre_max)/(-32767) 

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
    def on_R2_press(self,value):
        vit = map(value,-32252,32767,0,vitesse_max_m_s_soft*1000)
        if (vit < 0):
            set_vitesse_m_ms(0)
        else:
            set_vitesse_m_ms(vit)
        
 
    def on_L3_x_at_rest(self):
        set_direction_degre(90)
        
    def on_R1_press(self):
        set_vitesse_m_ms(0)
        
    def on_R1_release(self):
        set_vitesse_m_ms(0)
    
    def on_L3_right(self,value):
        print("x_r :", value, "degré : ",map(value,-32767, 32767, 60, 120))
        dir = map(value, 0, 32767, 90, 120)
        set_direction_degre(dir)

    def on_L3_left(self,value):
        dir = map(value,-32767, 0, 60, 90)
        print("x_l :", value)
        set_direction_degre(dir)
        
    def on_L2_press(self, value):
        vit = map(value,-32252,32767,0,vitesse_min_m_s_soft*1000)
        if (vit > 0):
            set_vitesse_m_ms(0)
        else:
            set_vitesse_m_ms(vit)
        
    # def on_R3_right(self, value):
    #     set_vitesse_m_s(value*a_prop)
        
    # def on_R3_left(self, value):
    #     set_vitesse_m_s(map(-32252,32767,0,vitesse_max_m_s_soft))




controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
try:
    Thread(target = envoie_direction_degre, daemon=True).start()
    controller.listen(timeout=60)

except KeyboardInterrupt:
    print("Arrêt du programme")
    pwm_prop.stop()
    pwm_dir.stop()
    controller.stop()
    exit(0)


