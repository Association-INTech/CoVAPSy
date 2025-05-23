from rpi_hardware_pwm import HardwarePWM
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.HL.Autotech_constant import PWM_PROP
# ...existing code...

#paramètres de la fonction vitesse_m_s, à étalonner 
direction_prop = PWM_PROP["direction_prop"] # 1 pour les variateurs à droite ou un petit rapport correspond à une marche avant
pwm_stop_prop = PWM_PROP["pwm_stop_prop"]
point_mort_prop = PWM_PROP["point_mort_prop"]
delta_pwm_max_prop = PWM_PROP["delta_pwm_max_prop"] #pwm à laquelle on atteint la vitesse maximale

vitesse_max_m_s_hard = 8 #vitesse que peut atteindre la voiture
vitesse_max_m_s_soft = 2 #vitesse maximale que l'on souhaite atteindre

pwm_prop = HardwarePWM(pwm_channel=0, hz=50, chip=2) #use chip 2 on pi 5 in accordance with the documentation
pwm_prop.start(pwm_stop_prop)

def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft :
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard :
        vitesse_m_s = -vitesse_max_m_s_hard
    if vitesse_m_s == 0 :
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0 :
        vitesse = vitesse_m_s * (delta_pwm_max_prop)/vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop*(point_mort_prop + vitesse ))
    elif vitesse_m_s < 0 :
        vitesse = vitesse_m_s * (delta_pwm_max_prop)/vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop - direction_prop*(point_mort_prop - vitesse ))
        
def recule():
    set_vitesse_m_s(-vitesse_max_m_s_hard)
    time.sleep(0.2)
    set_vitesse_m_s(0)
    time.sleep(0.2)
    set_vitesse_m_s(-1)

print("réglage des butées, Q pour quitter")
print("valeur numérique pour tester une vitesse en mm/s")
print("R pour reculer")
print("I pour inverser droite et gauche")
print("p pour diminuer delta_pwm_max_prop et P pour l'augmenter")
print("z pour diminuer le point zéro 1,5 ms et Z pour l'augmenter")
print("m pour diminuer le point mort et M pour l'augmenter")


while True :
    a = input("vitesse en mm/s, R, I, p, P, z, Z, m, M")
    try :
        vitesse_mm_s=int(a)
        set_vitesse_m_s(vitesse_mm_s/1000.0)
    except :        
        if a == "I" or a == "i" :
            direction_prop = -direction_prop
            print("nouvelle direction : " + str(direction_prop))
        elif a == "R" :
            recule()
            print("recule")
        elif a == "p" :
            delta_pwm_max_prop -=0.1
            print("nouveau delta_pwm_max_prop : " + str(delta_pwm_max_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop*(point_mort_prop+delta_pwm_max_prop))
        elif a == "P" :
            delta_pwm_max_prop +=0.1
            print("nouveau delta_pwm_max_prop : " + str(delta_pwm_max_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop*(point_mort_prop+delta_pwm_max_prop))
        elif a == "z" :
            pwm_stop_prop -=0.01
            print("nouveau pwm_stop_prop : " + str(pwm_stop_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop)
        elif a == "Z" :
            pwm_stop_prop +=0.01
            print("nouveau pwm_stop_prop : " + str(pwm_stop_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop)
        elif a == "m" :
            point_mort_prop -=0.01
            print("nouveau point_mort_prop : " + str(point_mort_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop*(point_mort_prop))
        elif a == "M" :
            point_mort_prop +=0.01
            print("nouveau point_mort_prop : " + str(point_mort_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop*(point_mort_prop))
        else :
            break

pwm_prop.change_duty_cycle(pwm_stop_prop)
print("nouvelles valeurs")
print("direction : "        + str(direction_prop))
print("delta_pwm_max_prop : "    + str(delta_pwm_max_prop))
print("point zero 1,5 ms : "+ str(pwm_stop_prop))
print("point mort : "       + str(point_mort_prop))