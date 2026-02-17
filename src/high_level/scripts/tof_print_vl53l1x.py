import time

import board
import busio
from adafruit_vl53l1x import VL53L1X

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the VL53L1X sensor
try:
    tof = VL53L1X(i2c)
    print("VL53L1X sensor initialized successfully.")
except Exception as e:
    print("Failed to initialize VL53L1X sensor:", e)
    exit(1)

# --- CONFIGURATION (important for VL53L1X) ---

# Distance mode:
# 1 = Short (~1.3m, faster, better precision)
# 2 = Long (~4m, a bit noisier)
tof.distance_mode = 2  # Long range (change if needed)

# Timing budget (ms)
# larger = more accurate, slower
tof.timing_budget = 50  # good compromise

# Start ranging
tof.start_ranging()

print("Starting continuous ranging...")

try:
    while True:
        if tof.data_ready:
            distance = tof.distance  # in cm
            print(f"Distance in cm: {distance}")
            tof.clear_interrupt()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping ranging...")
    tof.stop_ranging()
    print("Program terminated.")
