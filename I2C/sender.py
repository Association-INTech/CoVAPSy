from time import sleep
import smbus #type: ignore #ignore the module could not be resolved error because it is a linux only module
import struct

SLAVE_ADDRESS = 0x08
# Create an SMBus instance
bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

def write_data(float_data):
    # Convert the float to bytes
    print(float_data)
    byte_data = struct.pack('f', float_data)
    # Convert the bytes to a list of integers
    int_data = list(byte_data)
    print(int_data)
    # Write the data to the I2C bus
    bus.write_i2c_block_data(SLAVE_ADDRESS, 0, int_data[1:4])
    
def read_data(num_floats=3):
    # Each float is 4 bytes
    length = num_floats * 4
    # Read a block of data from the slave
    data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
    # Convert the byte data to floats
    if len(data) >= length:
        float_values = struct.unpack('f' * num_floats, bytes(data[:length]))
        return list(float_values)
    else:
        raise ValueError("Not enough data received from I2C bus")

i=0.56
while True :
    write_data(i)
    sleep(0.5)
    i+=1

