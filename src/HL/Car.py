import time
from rpi_hardware_pwm import HardwarePWM
import onnxruntime as ort
from scipy.special import softmax
import numpy as np
from gpiozero import LED, Button
import logging as log
import smbus # type: ignore #ignore the module could not be resolved error because it is a linux only module
import struct
from masterI2C import write_vitesse_direction

SLAVE_ADDRESS = 0x08
# Create an SMBus instance
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1


# Import constants from HL.Autotech_constant to share them between files and ease of use
from Autotech_constant import MAX_SOFT_SPEED, MAX_ANGLE, CRASH_DIST, MODEL_PATH, PWM_DIR, PWM_PROP, SOCKET_ADRESS, REAR_BACKUP_DIST,  LIDAR_DATA_SIGMA, LIDAR_DATA_AMPLITUDE, LIDAR_DATA_OFFSET
from Driver import Driver
from Lidar import Lidar
from Camera import Camera
from ToF import ToF

class Car:
    def __init__(self, driver):
        """Initialize the car's components."""
        self.vitesse_milimetres_s = 0  # Speed in millimeters per second
        self.angle_degre = 0  # Steering angle in degrees

        def _initialize_speed_limits():
            """Set the car's speed limits."""
            self.vitesse_max_m_s_hard = 6000  # Maximum hardware speed
            self.vitesse_max_m_s_soft = MAX_SOFT_SPEED  # Maximum software speed

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
            
        def _initialize_tof():
            """Initialize the ToF sensor."""
            try:
                self.tof = ToF()
                log.info("ToF initialized successfully")
            except Exception as e:
                log.error(f"Error initializing ToF: {e}")
                raise
        
        # Initialize speed limits
        _initialize_speed_limits()

        # Initialize AI session
        _initialize_ai()

        # Initialize Lidar
        _initialize_lidar()

        _initialize_camera()
        
        _initialize_tof()
        
        self.driving = driving_strategy
        
        

        log.info("Car initialization complete")

    def set_vitesse_m_s(self, vitesse_m_s):
        """Set the car's speed in meters per second."""
        self.vitesse_milimetres_s = int(vitesse_m_s * 1000)  # Convert to millimeters per second
        write_vitesse_direction(self.vitesse_milimetres_s,self.angle_degre) #take the vitesse in milimeters per second


    def set_direction_degre(self, n_angle_degre):
        """Set the car's steering angle in degrees."""
        self.angle_degre = n_angle_degre
        write_vitesse_direction(self.vitesse_milimetres_s, self.angle_degre) #take the angle in degrees
        
    
    def stop(self):
        self.vitesse_milimetres_s = 0
        self.angle_degre = 0
        write_vitesse_direction(self.vitesse_milimetres_s, self.angle_degre) #stop the car
        log.info("Arrêt du moteur")
        self.lidar.stop()
        

    def has_Crashed(self):
        
        small_distances = [d for d in self.lidar.rDistance[200:880] if 0 < d < CRASH_DIST] # 360 to 720 is the front of the car. 1/3 of the fov of the lidar
        log.debug(f"Distances: {small_distances}")
        if len(small_distances) > 2:
            # min_index = self.lidar.rDistance.index(min(small_distances))
            while self.tof.get_distance() < REAR_BACKUP_DIST:
                log.info(f"Obstacle arriere détecté {self.tof.get_distance()}")
                self.set_vitesse_m_s(0)
                time.sleep(0.1)
            return True
        return False

    def turn_around(self):
        """Turn the car around."""
        log.info("Turning around")
        
        self.set_vitesse_m_s(0)
        self.set_direction_degre(MAX_ANGLE)
        self.set_vitesse_m_s(-2) #blocing call
        time.sleep(1.8) # Wait for the car to turn around
        if self.camera.is_running_in_reversed():
            self.turn_around()



    def main(self):
        # récupération des données du lidar. On ne prend que les 1080 premières valeurs et on ignore la dernière par facilit" pour l'ia
        
        lidar_data = (self.lidar.rDistance[:1080]/1000)
        lidar_data_ai= (lidar_data-0.5)*(
            LIDAR_DATA_OFFSET + LIDAR_DATA_AMPLITUDE * np.exp(-1/2*((np.arange(1080) - 135) / LIDAR_DATA_SIGMA**2))
        ) #convertir en mètre et ajouter un bruit gaussien #On traffique les données fournit a l'IA
        angle, vitesse = self.driving(lidar_data_ai) #l'ai prend des distance en mètre et non en mm
        log.debug(f"Min Lidar: {min(lidar_data)}, Max Lidar: {max(lidar_data)}")
        self.set_direction_degre(angle)
        self.set_vitesse_m_s(vitesse)
        if self.camera.is_running_in_reversed():
            self.reverse_count += 1
        else:
            self.reverse_count = 0
        if self.reverse_count > 2:
            self.turn_around()
            self.reverse_count = 0
        if self.has_Crashed():
            print("Obstacle détecté")
            color= self.camera.is_green_or_red(lidar_data)
            if color == 0:
                small_distances = [d for d in self.lidar.rDistance if 0 < d < CRASH_DIST]
                if len(small_distances) == 0:
                    log.info("Aucun obstacle détecté")
                    return
                min_index = np.argmin(small_distances)
                direction = MAX_ANGLE if min_index < 540 else -MAX_ANGLE #540 is the middle of the lidar
                color = direction/direction
                log.info("Obstacle détecté, Lidar Fallback")
            if color == -1:
                log.info("Obstacle rouge détecté")
            if color == 1:
                log.info("Obstacle vert détecté")
            angle= -color*MAX_ANGLE
            self.set_vitesse_m_s(-2)
            self.set_direction_degre(angle)



if __name__ == '__main__':
    Format= '%(asctime)s:%(name)s:%(levelname)s:%(message)s'
    if input("Appuyez sur D pour démarrer en debug ou sur n'importe quelle autre touche pour démarrer en mode normal") in ("D", "d"):
        log.basicConfig(level=log.DEBUG, format=Format)
    else:
        log.basicConfig(level=log.INFO, format=Format)
    bp2 = Button("GPIO6")
    try:
        Schumacher = Driver(128, 128)
        GR86 = Car(Schumacher)
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
    