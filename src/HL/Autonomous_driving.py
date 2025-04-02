from HokuyoReader import HokuyoReader
import time
from rpi_hardware_pwm import HardwarePWM
import onnxruntime as ort
from scipy.special import softmax
import numpy as np
from gpiozero import LED, Button
import os


script_dir = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(script_dir, "model.onnx") #Allows the model to be loaded from the same directory as the script regardless of the current working directory (aka where the script is run from)
MAX_SOFT_SPEED = 0.25
MIN_SOFT_SPEED = 0.1
CRASH_DIST = 110  #mm




class Car():

    def __init__(self):

        self.IP = '192.168.0.10'
        self.PORT = 10940

        # Paramètres de la fonction vitesse_m_s
        # -1 pour les variateurs inversés ou un petit rapport correspond à une marche avant
        self.direction_prop = 1
        self.pwm_stop_prop = 7.53
        self.point_mort_prop = 0.5
        # pwm à laquelle on atteint la vitesse maximale
        self.delta_pwm_max_prop = 1.1

        # vitesse que peut atteindre la voiture
        self.vitesse_max_m_s_hard = 8
        # vitesse maximale que l'on souhaite atteindre
        self.vitesse_max_m_s_soft = MAX_SOFT_SPEED

        # Paramètres de la fonction set_direction_degre
        self.direction = -1  # 1 pour angle_pwm_min a gauche, -1 pour angle_pwm_min à droite
        # min
        self.angle_pwm_min = 6.91
        # max
        self.angle_pwm_max = 10.7
        self.angle_pwm_centre = 8.805

        # vers la gauche
        self.angle_degre_max = +18
        self.angle_degre = 0

        # Initialisation des pwm

        # Utilisation du chip 2 sur la pi 5 pour correspondre à la documentation
        self.pwm_prop = HardwarePWM(pwm_channel=0, hz=50, chip=2)
        self.pwm_prop.start(self.pwm_stop_prop)

        # Utilisation du chip 2 sur la pi 5 pour correspondre à la documentation
        self.pwm_dir = HardwarePWM(pwm_channel=1, hz=50, chip=2)
        self.pwm_dir.start(self.angle_pwm_centre)
        self.ai_session = ort.InferenceSession(MODEL_PATH)
        print("pwm loaded")

        self.lookup_dir = np.linspace(-18, 18, 16)
        self.lookup_prop = np.linspace(MIN_SOFT_SPEED, MAX_SOFT_SPEED, 16)

        # Initialisation du lidar

        self.lidar = HokuyoReader(self.IP, self.PORT)
        self.lidar.stop()
        self.lidar.startContinuous(0, 1080)

    def set_vitesse_m_s(self, vitesse_m_s):
        if vitesse_m_s > self.vitesse_max_m_s_soft:
            vitesse_m_s = self.vitesse_max_m_s_soft
        elif vitesse_m_s < -self.vitesse_max_m_s_hard:
            vitesse_m_s = -self.vitesse_max_m_s_hard
        if vitesse_m_s == 0:
            self.pwm_prop.change_duty_cycle(self.pwm_stop_prop)
        elif vitesse_m_s > 0:
            vitesse = vitesse_m_s * \
                (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard
            self.pwm_prop.change_duty_cycle(
                self.pwm_stop_prop + self.direction_prop*(self.point_mort_prop + vitesse))
        elif vitesse_m_s < 0:
            vitesse = vitesse_m_s * \
                (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard
            self.pwm_prop.change_duty_cycle(
                self.pwm_stop_prop - self.direction_prop*(self.point_mort_prop - vitesse))

    def recule(self):
        self.set_vitesse_m_s(-self.vitesse_max_m_s_hard)
        time.sleep(0.2)
        self.set_vitesse_m_s(0)
        time.sleep(0.2)
        self.set_vitesse_m_s(-2)
        time.sleep(1)

    def set_direction_degre(self, angle_degre):
        angle_pwm = self.angle_pwm_centre + self.direction * \
            (self.angle_pwm_max - self.angle_pwm_min) * \
            angle_degre / (2 * self.angle_degre_max)
        if angle_pwm > self.angle_pwm_max:
            angle_pwm = self.angle_pwm_max
        if angle_pwm < self.angle_pwm_min:
            angle_pwm = self.angle_pwm_min
        self.pwm_dir.change_duty_cycle(angle_pwm)

    def stop(self):
        self.pwm_dir.stop()
        self.pwm_prop.start(self.pwm_stop_prop)
        print("Arrêt du moteur")
        self.lidar.stop()
        # exit() #not to be used in prodution/library? https://www.geeksforgeeks.org/python-exit-commands-quit-exit-sys-exit-and-os-_exit/
    
    def has_Crashed(self):
        small_indices = []
        small_distances = []
        min_distance = float("inf")
        min_index = None
    
        for index, distance in enumerate(self.lidar.rDistance):
            if 0 < distance < CRASH_DIST:
                small_indices.append(index)
                small_distances.append(distance)
            if 0 < distance < min_distance:
                min_distance = distance
                min_index = index
    
        if len(small_distances) > 2:
            direction = 18 if min_index < 540 else -18
            self.set_direction_degre(-direction) #TODO: change when camera backup is implemented
            return True
        return False

    def ai_update(self, lidar_data):
        lidar_data = np.array(lidar_data, dtype=np.float32)
        # 2 vectors direction and speed. direction is between hard left at index 0 and hard right at index 1. speed is between min speed at index 0 and max speed at index 1
        vect = self.ai_session.run(None, {'input': lidar_data[None]})[0][0]

        vect_dir, vect_prop = vect[:16], vect[16:]  # split the vector in 2
        vect_dir = softmax(vect_dir)  # distribution de probabilité
        vect_prop = softmax(vect_prop)

        angle = sum(self.lookup_dir*vect_dir)  # moyenne pondérée des angles
        # moyenne pondérée des vitesses
        vitesse = sum(self.lookup_prop*vect_prop)

        return angle, vitesse

    def main(self):
        # récupération des données du lidar. On ne prend que les 1080 premières valeurs et on ignore la dernière par facilit" pour l'ia
        
        lidar_data = self.lidar.rDistance[:1080]
        angle, vitesse = self.ai_update(lidar_data)
        self.set_direction_degre(angle)
        self.set_vitesse_m_s(vitesse)
        if self.has_Crashed():
            self.recule()



if __name__ == '__main__':
    bp2 = Button("GPIO6")
    try:
        GR86 = Car()
        print("Initialisation terminée")
        if input("Appuyez sur D pour démarrer ou tout autre touche pour quitter") in ("D", "d") or bp2.is_pressed:
            print("Depart")
            while True:
                GR86.main()
        else:
            raise KeyboardInterrupt
    except KeyboardInterrupt:
        GR86.stop()
        print("Programme arrêté par l'utilisateur")
        
    except Exception as e: # catch all exceptions to stop the car
        GR86.stop()
        print("Erreur inconnue")
        raise e # re-raise the exception to see the error message
    