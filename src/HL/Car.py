import time
from rpi_hardware_pwm import HardwarePWM
import onnxruntime as ort
from scipy.special import softmax
import numpy as np
from gpiozero import LED, Button
import logging as log


# Import constants from HL.Autotech_constant to share them between files and ease of use
from Autotech_constant import MAX_SOFT_SPEED, MAX_ANGLE, CRASH_DIST, MODEL_PATH, PWM_DIR, PWM_PROP, SOCKET_ADRESS
from Driver import Driver
from Lidar import Lidar
from Camera import Camera

class Car:
    def __init__(self, driving_strategy=Driver().farthest_distants):
        """Initialize the car's components."""
        
        def _initialize_speed_limits():
            """Set the car's speed limits."""
            self.vitesse_max_m_s_hard = 8  # Maximum hardware speed
            self.vitesse_max_m_s_soft = MAX_SOFT_SPEED  # Maximum software speed

        def _initialize_pwm():
            """Initialize PWM components for propulsion and steering."""
            try:
                # Load parameters from PWM_DIR
                self.direction_prop = PWM_DIR["direction_prop"]
                self.pwm_stop_prop = PWM_DIR["pwm_stop_prop"]
                self.point_mort_prop = PWM_DIR["point_mort_prop"]
                self.delta_pwm_max_prop = PWM_DIR["delta_pwm_max_prop"]

                # Load parameters from PWM_PROP
                self.direction = PWM_PROP["direction"]
                self.angle_pwm_min = PWM_PROP["angle_pwm_min"]
                self.angle_pwm_max = PWM_PROP["angle_pwm_max"]
                self.angle_pwm_centre = PWM_PROP["angle_pwm_centre"]

                # Initialize propulsion PWM
                self.pwm_prop = HardwarePWM(pwm_channel=0, hz=50, chip=2)
                self.pwm_prop.start(self.pwm_stop_prop)

                # Initialize steering PWM
                self.pwm_dir = HardwarePWM(pwm_channel=1, hz=50, chip=2)
                self.pwm_dir.start(self.angle_pwm_centre)

                log.info("PWM initialized successfully")
            except Exception as e:
                log.error(f"Error initializing PWM: {e}")
                raise

        def _initialize_ai():
            """Initialize the AI session."""
            try:
                self.ai_session = ort.InferenceSession(MODEL_PATH)
                log.info("AI session initialized successfully")
            except Exception as e:
                log.error(f"Error initializing AI session: {e}")
                raise

        def _initialize_lidar():
            """Initialize the Lidar sensor."""
            try:
                self.lidar = Lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
                self.lidar.stop()
                self.lidar.startContinuous(0, 1080)
                log.info("Lidar initialized successfully")
            except Exception as e:
                log.error(f"Error initializing Lidar: {e}")
                raise
        
        def _initialize_camera():
            """Initialize the camera."""
            try:
                self.reverse_count = 0
                self.camera = Camera()
                self.camera.start()
                time.sleep(0.2)  # Allow time for the camera to start
                log.info("Camera initialized successfully")
            except Exception as e:
                log.error(f"Error initializing Camera: {e}")
                raise
        
        # Initialize speed limits
        _initialize_speed_limits()

        # Initialize PWM components
        _initialize_pwm()

        # Initialize AI session
        _initialize_ai()

        # Initialize Lidar
        _initialize_lidar()
        
        _initialize_camera()
        
        self.driving = driving_strategy
        
        

        log.info("Car initialization complete")

    def set_vitesse_m_s(self, vitesse_m_s):
        """Set the car's speed in meters per second."""
        # Clamp the speed to the maximum and minimum speed
        vitesse_m_s = max(-self.vitesse_max_m_s_hard, min(vitesse_m_s, self.vitesse_max_m_s_soft)) 
        vitesse_pwm = vitesse_m_s * (self.delta_pwm_max_prop)/self.vitesse_max_m_s_hard
        
        if vitesse_m_s == 0:
            pwm = self.pwm_stop_prop
        elif vitesse_m_s > 0:
            pwm= self.pwm_stop_prop + self.direction_prop*(self.point_mort_prop + vitesse_pwm)
            
        elif vitesse_m_s < 0:
            pwm= self.pwm_stop_prop - self.direction_prop*(self.point_mort_prop - vitesse_pwm)
            
        self.pwm_prop.change_duty_cycle(pwm)
        log.debug(f"Vitesse: {vitesse_m_s} m/s, PWM: {pwm}")


    def set_direction_degre(self, angle_degre):
        """Set the car's steering angle in degrees."""
        angle_pwm = self.angle_pwm_centre + self.direction * ((self.angle_pwm_max - self.angle_pwm_min) * angle_degre / (2 * MAX_ANGLE))
        
        # Clamp the angle to the maximum and minimum angle
        angle_pwm = max(self.angle_pwm_min, min(angle_pwm, self.angle_pwm_max))
        log.debug(f"Angle: {angle_degre}°, PWM: {angle_pwm}")
        self.pwm_dir.change_duty_cycle(angle_pwm)
        
    def recule(self):
        """Set the car to reverse."""
        self.set_vitesse_m_s(-self.vitesse_max_m_s_hard)
        time.sleep(0.2)
        self.set_vitesse_m_s(0)
        time.sleep(0.2)
        self.set_vitesse_m_s(-1)
    
    def stop(self):
        self.pwm_dir.stop()
        self.pwm_prop.start(self.pwm_stop_prop)
        log.info("Arrêt du moteur")
        self.lidar.stop()
        # exit() #not to be used in prodution/library? https://www.geeksforgeeks.org/python-exit-commands-quit-exit-sys-exit-and-os-_exit/
    
    def has_Crashed(self):
        small_distances = [d for d in self.lidar.rDistance if 0 < d < CRASH_DIST]
        log.debug(f"Distances: {small_distances}")
        if len(small_distances) > 2:
            # min_index = self.lidar.rDistance.index(min(small_distances))
            return True
        return False
    
    def turn_around(self):
        """Turn the car around."""
        self.set_vitesse_m_s(0)
        self.set_direction_degre(MAX_ANGLE)
        self.recule() #blocing call
        self.set_vitesse_m_s(0)
        self.set_direction_degre(-MAX_ANGLE)
        self.set_vitesse_m_s(MAX_SOFT_SPEED*0.25)
        time.sleep(1)
        self.set_vitesse_m_s(0)


    def main(self):
        # récupération des données du lidar. On ne prend que les 1080 premières valeurs et on ignore la dernière par facilit" pour l'ia
        
        lidar_data = self.lidar.rDistance[:1080]
        angle, vitesse = self.driving(lidar_data)
        self.set_direction_degre(angle)
        self.set_vitesse_m_s(vitesse)
        if self.camera.is_running_in_reversed():
            self.reverse_count += 1
        else:
            self.reverse_count = 0
        if self.reverse_count > 3:
            self.turn_around()
            self.reverse_count = 0
        if self.has_Crashed():
            angle= self.camera.is_green_or_red()*MAX_ANGLE
            self.set_direction_degre(angle)
            self.recule()



if __name__ == '__main__':
    log.basicConfig(level=log.INFO)
    bp2 = Button("GPIO6")
    try:
        Schumacher = Driver()
        GR86 = Car(Schumacher.ai)
        log.info("Initialisation terminée")
        if input("Appuyez sur D pour démarrer ou tout autre touche pour quitter") in ("D", "d") or bp2.is_pressed:
            log.info("Depart")
            while True:
                GR86.main()
        else:
            raise Exception("Le programme a été arrêté par l'utilisateur")
    except KeyboardInterrupt:
        GR86.stop()
        log.info("Le programme a été arrêté par l'utilisateur")
    
    except Exception as e: # catch all exceptions to stop the car
        GR86.stop()
        log.error("Erreur inconnue")
        raise e # re-raise the exception to see the error message
    