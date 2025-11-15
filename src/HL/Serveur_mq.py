import zmq
context = zmq.Context()

import time
import threading
import smbus
import logging as log
#on démarre les log
log.basicConfig(level=log.INFO, format=Format)

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08

length_i2c_received = 3 #le nombre de donnée récupéré par l'i2c

# on recoit les inoformations
received = context.socket(zmq.REP)
received.bind("tcp://0.0.0.0:5555")
# on envoie les informations
send = context.socket(zmq.PUB)
send.bind("tcp://0.0.0.0:5556")

vitesse_d = 0
vitesse_r = 0
direction = 0

voltage_lipo = 0
voltage_nimh = 0


def i2c_loop():
    """Envoie vitesse/direction régulièrement au microcontroleur."""
    global vitesse_d, direction

    while True:
        try : 
            data = struct.pack('<ff', float(vitesse_d), float(direction))
            bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
            time.sleep(0.05)
        except :
            print("i2c mort")
            time.sleep(1)

def i2c_received():
    
    global voltage_lipo, voltage_nimh, vitesse_r, length_i2c_received
    length = length_i2c_received * 4 
    while True:
        data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
        # Convert the byte data to a float
        if len(data) >= length:
            float_values = struct.unpack('f' * length_i2c_received, bytes(data[:length]))
            list_valeur = list(float_values)

            # on enregistre les valeur
            voltage_lipo = list_valeur[0]
            voltage_nimh = list_valeur[1]
            vitesse_r = list_valeur[2]

        else:
            pass


def msg_received():
    global vitesse_d, direction
    while True :
        req = received.recv_json()

        if req["cmd"] == "set_speed":
            vitesse_d = req["value"]
            received.send_json({"status": "ok"})
        elif req["cmd"] == "set_direction":
            direction = req["value"]
            received.send_json({"status": "ok"})
        else:
            received.send_json({"error": "unknown"})

if __name__ == "__main__":
    threading.Thread(target=i2c_loop, daemon=True).start()
    threading.Thread(target=i2c_received, daemon=True).start()

    msg_received()

