import logging
import struct
import threading
import time

import smbus  # type: ignore #ignore the module could not be resolved error because it is a linux only module

from high_level.autotech_constant import (
    I2C_NUMBER_DATA_RECEIVED,
    I2C_SLEEP_ERROR_LOOP,
    I2C_SLEEP_RECEIVED,
    SLAVE_ADDRESS,
)


class I2CArduino:
    def __init__(self, serveur):
        self.log = logging.getLogger(__name__)
        self.serveur = serveur
        self.current_speed = 0
        self.send_running = True
        self.receive_running = True

        # battery info
        self.voltage_lipo = 0
        self.voltage_nimh = 0

        # initialisation of i2c bus
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
        self.log.info("I2C: bus opened on /dev/i2c-1")

        # initialization of i2c send and received
        threading.Thread(target=self.start_send, daemon=True).start()
        threading.Thread(target=self.start_received, daemon=True).start()

    def start_send(self):
        """send speed and direction to the microcontroller regularly."""
        time.sleep(1)  # Give some time for the target_speed and direction to be set
        self.log.info("Thread I2C loop started")
        while self.send_running:
            try:
                data = struct.pack(
                    "<ff",
                    float(self.serveur.target_speed),
                    float(self.serveur.direction),
                )
                self.bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                time.sleep(1e-4)  # Short delay to prevent overwhelming the bus
            except Exception as e:
                self.log.error("Erreur I2C write: %s", e, exc_info=True)
                time.sleep(I2C_SLEEP_ERROR_LOOP)

    def start_received(self):
        """received data from the microcontroller regularly."""
        self.log.info("Thread I2C receive started")
        length = I2C_NUMBER_DATA_RECEIVED * 4
        while self.receive_running:
            data = self.bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
            # Convert the byte data to a float
            if len(data) >= length:
                float_values = struct.unpack(
                    "f" * I2C_NUMBER_DATA_RECEIVED, bytes(data[:length])
                )
                list_values = list(float_values)

                # on enregistre les valeur
                self.voltage_lipo = list_values[0]
                self.voltage_nimh = list_values[1]
                self.current_speed = list_values[2]
            else:
                self.log.warning(
                    "I2C: unexpected size (%d but %d excepted)", len(data), length
                )
            time.sleep(I2C_SLEEP_RECEIVED)
