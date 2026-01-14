import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import time
import struct
from Autotech_constant import I2C_SLEEP_RECEIVED, I2C_NUMBER_DATA_RECEIVED, I2C_SLEEP_ERROR_LOOP, SLAVE_ADDRESS
import logging
import threading

class I2c_arduino:
    def __init__(self,serveur):
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
        self.vitesse_r = 0
        self.send_running = True
        self.receive_running = True
        
        #voltage des lipos
        self.voltage_lipo = 0
        self.voltage_nimh = 0

        #initialisation du bus i2c
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
        self.log.info("I2C: bus ouvert sur /dev/i2c-1")

        time.sleep(0.5)  # Give some time for the bus to settle

        #initialization of i2c send and received
        threading.Thread(target=self.start_send, daemon=True).start()
        threading.Thread(target=self.start_received, daemon=True).start()
    
    def start_send(self):
        """Envoie vitesse/direction régulièrement au microcontroleur. (toute les frames actuellement)"""
        self.log.info("Thread I2C loop démarré")
        while self.send_running:
            try :
                data = struct.pack('<ff', float(round(self.serveur.vitesse_d)), float(round(self.serveur.direction_d)))
                self.bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                time.sleep(1e-5) # Short delay to prevent overwhelming the bus
            except Exception as e:
                self.log.error("Erreur I2C write: %s", e, exc_info=True)
                time.sleep(I2C_SLEEP_ERROR_LOOP)

    def start_received(self):
        """récupére les informations de l'arduino"""
        self.log.info("Thread I2C receive démarré")
        length = I2C_NUMBER_DATA_RECEIVED * 4 
        while self.receive_running:
            data = self.bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
            # Convert the byte data to a float
            if len(data) >= length:
                float_values = struct.unpack('f' * I2C_NUMBER_DATA_RECEIVED, bytes(data[:length]))
                list_valeur = list(float_values)

                # on enregistre les valeur
                self.voltage_lipo = list_valeur[0]
                self.voltage_nimh = list_valeur[1]
                self.vitesse_r = list_valeur[2]
            else:
                self.log.warning("I2C: taille inattendue (%d au lieu de %d)", len(data), length)
            time.sleep(I2C_SLEEP_RECEIVED)