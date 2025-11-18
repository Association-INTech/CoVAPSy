import pygame
import zmq
import time
from threading import Thread

###################################################
# Init ZMQ
###################################################
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://192.168.1.10:5556")

def envoie_donnee():
    global vitesse_m, direction_d
    while True:
        socket.send_json({"cmd": "set_speed", "value": vitesse_m})
        socket.send_json({"cmd": "set_direction", "value": direction_d})
        time.sleep(0.01)

###################################################
# Paramètres véhicule
###################################################
direction_d = 0
vitesse_m = 0

vitesse_max_m_s_soft = 2
vitesse_min_m_s_soft = -2
angle_degre_max = 18

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def set_direction_degre(angle_degre):
    global direction_d
    direction_d = angle_degre
    print(direction_d, vitesse_m)

def set_vitesse_m_ms(vit):
    global vitesse_m
    vitesse_m = vit

###################################################
# Init pygame + manette
###################################################
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Aucune manette détectée")
    exit(1)

joy = pygame.joystick.Joystick(0)
joy.init()
print("Manette détectée:", joy.get_name())

###################################################
# Boucle principale
###################################################
Thread(target=envoie_donnee, daemon=True).start()

try:
    while True:
        pygame.event.pump()

        # Axes :
        # Pour Xbox/PS4 USB :
        # L2 = axis 2   (souvent 0..1)
        # R2 = axis 5   (souvent 0..1)
        # Stick gauche horizontal = axis 0 (-1..1)

        axis_lx = joy.get_axis(0)         # Gauche droite
        axis_l2 = joy.get_axis(2)         # Accélération inverse
        axis_r2 = joy.get_axis(5)         # Accélération

        # Direction
        direction = map_range(axis_lx, -1, 1, -angle_degre_max, angle_degre_max)
        set_direction_degre(direction)

        # Accélération
        accel = (axis_r2 + 1)/2
        brake = (axis_l2 + 1)/2

        # Certaines manettes vont de -1..1, d'autres 0..1

        # Avant
        if accel > 0.05:
            vit = accel * vitesse_max_m_s_soft * 1000
            set_vitesse_m_ms(vit)

        # Arrière
        elif brake > 0.05:
            vit = brake * vitesse_min_m_s_soft * 1000
            set_vitesse_m_ms(vit)
        else :
            set_vitesse_m_ms(0)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Fin du programme.")
    pygame.quit()
