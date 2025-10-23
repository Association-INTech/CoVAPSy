from pyPS4Controller.controller import Controller
import time
from masterI2C import write_vitesse_direction

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

#on recode map de arduino pour des soucis 

def map(x, in_min,in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def set_direction_degre(angle_degre) :
    global direction_d
    direction_d = angle_degre
    write_vitesse_direction(vitesse_m, direction_d) #take the angle in degrees
    
def set_vitesse_m_s(vitesse_m_s):
    global vitesse_m
    vitesse_m = vitesse_m_s
    write_vitesse_direction(int(vitesse_m_s * 1000), direction_d) # Convert to millimeters per second
        
def recule():
    global vitesse_m
    vitesse_m = -2
    write_vitesse_direction(int(vitesse_m * 1000), direction_d) # Convert to millimeters per second


a_prop = vitesse_max_m_s_soft/(65198)
a_dir=(angle_degre_max)/(-32767) 

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        
    def on_R2_press(self,value):
         print("La valeur de R2 est: ",value)
         set_vitesse_m_s(map(value,-32252,32767,0,vitesse_max_m_s_soft))
        
    def on_R2_release(self):
         print("Arrêt complet")
         set_vitesse_m_s(0)
 
    def on_L3_x_at_rest(self):
        set_direction_degre(0)
        
    def on_R1_press(self):
        recule()
        
    def on_R1_release(self):
        set_vitesse_m_s(0)
    
    def on_L3_right(self,value):
        print("x_r :", value)
        set_direction_degre(90-a_dir*value)

    def on_L3_left(self,value):
        print("x_l :", value)
        set_direction_degre(90-a_dir*value)
        
    def on_L2_press(self, value):
         set_vitesse_m_s(map(value,-32252,32767,0,vitesse_min_m_s_soft))
        
    # def on_R3_right(self, value):
    #     set_vitesse_m_s(value*a_prop)
        
    # def on_R3_left(self, value):
    #     set_vitesse_m_s(map(-32252,32767,0,vitesse_max_m_s_soft))




controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
try:
    controller.listen()
    print("hello world")
    print("hello world")
    print("hello world")

except KeyboardInterrupt:
    print("Arrêt du programme")
    pwm_prop.stop()
    pwm_dir.stop()
    controller.stop()
    exit(0)


