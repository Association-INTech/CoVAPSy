import time
import onnxruntime as ort
import numpy as np
import logging
from threading import Thread
from .program import Program
import logging
# Import constants from HL.Autotech_constant to share them between files and ease of use
from high_level.autotech_constant import  MAX_ANGLE, CRASH_DIST, MODEL_PATH, REAR_BACKUP_DIST,  LIDAR_DATA_SIGMA, LIDAR_DATA_AMPLITUDE, LIDAR_DATA_OFFSET
from .utils.driver import Driver


class Car:
    def __init__(self,driving_strategy, serveur):
        self.log = logging.getLogger(__name__)

        """Initialize the car's components."""
        self.target_speed = 0  # Speed in millimeters per second
        self.direction = 0  # Steering angle in degrees
        self.serveur = serveur
        self.reverse_count = 0
        def _initialize_ai():
            """Initialize the AI session."""
            try:
                self.ai_session = ort.InferenceSession(MODEL_PATH)
                self.log.info("AI session initialized successfully")
            except Exception as e:
                self.log.error(f"Error initializing AI session: {e}")
                raise
        # Initialize AI session
        _initialize_ai()
        
        self.driving = driving_strategy

        self.log.info("Car initialization complete")
    # accès dynamique aux capteurs
    @property
    def camera(self):
        return self.serveur.camera

    @property
    def lidar(self):
        return self.serveur.lidar

    @property
    def tof(self):
        return self.serveur.tof


    def stop(self):
        self.target_speed = 0
        self.direction = 0
        self.log.info("Arrêt du moteur")
        

    def has_Crashed(self):
        
        small_distances = [d for d in self.lidar.rDistance[200:880] if 0 < d < CRASH_DIST] # 360 to 720 is the front of the car. 1/3 of the fov of the lidar
        self.log.debug(f"Distances: {small_distances}")
        if len(small_distances) > 2:
            # min_index = self.lidar.rDistance.index(min(small_distances))
            while self.tof.get_distance() < REAR_BACKUP_DIST:
                self.log.info(f"Obstacle arriere détecté {self.tof.get_distance()}")
                self.target_speed = 0
                time.sleep(0.1)
            return True
        return False

    def turn_around(self):
        """Turn the car around."""
        self.log.info("Turning around")
        
        self.target_speed = 0
        self.direction = MAX_ANGLE
        self.target_speed = -2 #blocing call
        time.sleep(1.8) # Wait for the car to turn around
        if self.camera.is_running_in_reversed():
            self.turn_around()



    def main(self):
        # récupération des données du lidar. On ne prend que les 1080 premières valeurs et on ignore la dernière par facilit" pour l'ia
        if self.camera is None or self.lidar is None :
            self.log.debug("Capteurs pas encore prêts")
            print("Capteurs pas encore prêts")
            return
        lidar_data = (self.lidar.rDistance[:1080]/1000)
        lidar_data_ai= (lidar_data-0.5)*(
            LIDAR_DATA_OFFSET + LIDAR_DATA_AMPLITUDE * np.exp(-1/2*((np.arange(1080) - 135) / LIDAR_DATA_SIGMA**2))
        ) #convertir en mètre et ajouter un bruit gaussien #On traffique les données fournit a l'IA
        self.direction, self.target_speed = self.driving(lidar_data_ai
                                                         #,self.camera.camera_matrix()
                                                         ) #l'ai prend des distance en mètre et non en mm
        self.log.debug(f"Min Lidar: {min(lidar_data)}, Max Lidar: {max(lidar_data)}")

        if self.camera.is_running_in_reversed():
            self.reverse_count += 1
        else:
            self.reverse_count = 0
        if self.reverse_count > 2:
            self.turn_around()
            self.reverse_count = 0
            """
        if self.has_Crashed():
            print("Obstacle détecté")
            color= self.camera.is_green_or_red(lidar_data)
            if color == 0:
                small_distances = [d for d in self.lidar.rDistance if 0 < d < CRASH_DIST]
                if len(small_distances) == 0:
                    self.log.info("Aucun obstacle détecté")
                    return
                min_index = np.argmin(small_distances)
                direction = MAX_ANGLE if min_index < 540 else -MAX_ANGLE #540 is the middle of the lidar
                color = direction/direction
                self.log.info("Obstacle détecté, Lidar Fallback")
            if color == -1:
                self.log.info("Obstacle rouge détecté")
            if color == 1:
                self.log.info("Obstacle vert détecté")
            angle= -color*MAX_ANGLE
            self.target_speed = -2
            self.direction = angle"""




class Ai_Programme(Program):
    def __init__(self, serveur):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
        self.driver = None
        self.GR86 = None
        self.running = False
        self.controls_car = True

    @property
    def target_speed(self):
        if self.GR86 == None:
            return 0
        return self.GR86.target_speed
    
    @property
    def direction(self):
        if self.GR86 == None:
            return 0
        return self.GR86.direction

    def run(self):
        while self.running:
            try:
                self.GR86.main()
            except Exception as e:
                self.log.error(f"Erreur IA: {e}")
                self.running = False
                raise
    
    def start(self):
        if self.running:
            return

        if self.serveur.camera is None or self.serveur.lidar is None:
            print("Capteurs non initialisés")
            return

        try:
            self.driver = Driver(128, 128)
            self.driver.load_model()
            self.GR86 = Car(self.driver.ai, self.serveur)
        except Exception as e:
            self.log.error(f"Impossible de démarrer l'IA: {e}")
            self.driver = None
            self.GR86 = None
            return

        self.running = True
        Thread(target=self.run, daemon=True).start()

    
    def stop(self):
        self.running = False
        self.GR86.stop()

"""
if __name__ == '__main__': # non fonctionnelle
    Format= '%(asctime)s:%(name)s:%(levelname)s:%(message)s'
    if input("Appuyez sur D pour démarrer en debug ou sur n'importe quelle autre touche pour démarrer en mode normal") in ("D", "d"):
        logging.basicConfig(level=logging.DEBUG, format=Format)
    else:
        logging.basicConfig(level=logging.INFO, format=Format)
    bp2 = Button("GPIO6")
    try:
        Schumacher = Driver(128, 128)
        GR86 = Car(Schumacher,None,None)
        GR86._initialize_camera()
        GR86._initialize_lidar()
        logging.info("Initialisation terminée")
        if input("Appuyez sur D pour démarrer ou tout autre touche pour quitter") in ("D", "d") or bp2.is_pressed:
            logging.info("Depart")
            while True:
                GR86.main()
        else:
            raise Exception("Le programme a été arrêté par l'utilisateur")
    except KeyboardInterrupt:
        GR86.stop()
        logging.info("Le programme a été arrêté par l'utilisateur")

    except Exception as e: # catch all exceptions to stop the car
        GR86.stop()
        logging.error("Erreur inconnue")
        raise e # re-raise the exception to see the error message
    """