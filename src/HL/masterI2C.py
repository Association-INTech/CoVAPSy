import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import time
import struct

# I2C address of the slave
SLAVE_ADDRESS = 0x08

class I2c_arduino:
    def __init__(self,serveur):
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
        self.vitesse_r = 0
        self.length_i2c_received = 3
        self.send_running = True
        self.receive_running = True
        self.time_between_received = 0.1
        
        #voltage des lipos
        self.voltage_lipo = 0
        self.voltage_nimh = 0

        #initialisation du bus i2c
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
        self.log.info("I2C: bus ouvert sur /dev/i2c-1")

        #initialization of i2c send and received
        threading.Thread(target=self.start_send(), daemon=True).start()
        threading.Thread(target=self.start_received(), daemon=True).start()
    
    def start_send(self):
        """Envoie vitesse/direction régulièrement au microcontroleur. (toute les frames actuellement)"""
        self.log.info("Thread I2C loop démarré")
        while self.send_running:
            try :
                data = struct.pack('<ff', float(round(self.serveur.programme[self.serveur.last_programme_control].vitesse_d)), float(round(self.serveur.programme[self.last_programme_control].direction_d)))
                self.bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
            except Exception as e:
                self.log.error("Erreur I2C write: %s", e, exc_info=True)
                time.sleep(1)

    def start_received(self):
        """récupére les informations de l'arduino"""
        self.log.info("Thread I2C receive démarré")
        length = self.length_i2c_received * 4 
        while self.receive_running:
            data = self.bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
            # Convert the byte data to a float
            if len(data) >= length:
                float_values = struct.unpack('f' * self.length_i2c_received, bytes(data[:length]))
                list_valeur = list(float_values)

                # on enregistre les valeur
                self.voltage_lipo = list_valeur[0]
                self.voltage_nimh = list_valeur[1]
                self.vitesse_r = list_valeur[2]
            else:
                self.log.warning("I2C: taille inattendue (%d au lieu de %d)", len(data), length)
            time.sleep(self.time_between_received)