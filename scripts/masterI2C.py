import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import time
import numpy as np
import struct
# Create an SMBus instance
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08


vitesse = 200 # en millimetre par seconde
direction = 100 # en degré


def write_data(vitesse,direction):
    # Convert string to list of ASCII values
    data = struct.pack('<ff', float(vitesse), float(direction))
    bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))

import struct

def read_data(length):
    # Read a block of data from the slave
    data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
    # Convert the byte data to a float
    if len(data) >= 4:
        float_value = struct.unpack('f', bytes(data[:4]))[0]
        return float_value
    else:
        raise ValueError("Not enough data received from I2C bus")

if __name__ == "__main__":
    try:
        # Send data to the slave
        while(True):
            vitesse= float(input("vitesse en millimetre par seconde:"))
            rotation= float(input("rotation en degré:"))
            write_data(vitesse,rotation)
            time.sleep(0.1)  # Wait for the slave to process the data
            received = read_data(8)  # Adjust length as needed
            print("Received from slave:", received)

        # Request data from the slave
        

    except Exception as e:
        print("Error:", e)