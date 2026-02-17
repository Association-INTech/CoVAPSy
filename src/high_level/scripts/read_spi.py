import struct
import time

import spidev  # type: ignore #ignore the module could not be resolved error because it is a linux only module

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device (CS) 0
spi.max_speed_hz = 50000


def read_voltage():
    # Send a dummy byte to initiate SPI communication
    response = spi.xfer2([0x00] * 8)  # 8 bytes to read two float values (4 bytes each)

    # Convert the received bytes to float values
    voltage_lipo = struct.unpack("f", bytes(response[0:4]))[0]
    voltage_nimh = struct.unpack("f", bytes(response[4:8]))[0]

    return voltage_lipo, voltage_nimh


try:
    while True:
        voltage_lipo, voltage_nimh = read_voltage()
        print(f"LiPo Voltage: {voltage_lipo:.2f} V, NiMh Voltage: {voltage_nimh:.2f} V")
        time.sleep(1)  # Adjust the delay as needed

except KeyboardInterrupt:
    spi.close()
    print("SPI communication closed.")
