# #include "Adafruit_VL53L0X.h"

# Adafruit_VL53L0X lox = Adafruit_VL53L0X();

# void setup() {
#   Serial.begin(115200);

#   // wait until serial port opens for native USB devices
#   while (! Serial) {
#     delay(1);
#   }

#   Serial.println("Adafruit VL53L0X test.");
#   if (!lox.begin()) {
#     Serial.println(F("Failed to boot VL53L0X"));
#     while(1);
#   }
#   // power
#   Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

#   // start continuous ranging
#   lox.startRangeContinuous();
# }

# void loop() {
#   if (lox.isRangeComplete()) {
#     Serial.print("Distance in mm: ");
#     Serial.println(lox.readRange());
#   }
# }

import time
import board
import busio
from adafruit_vl53l0x import VL53L0X

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the VL53L0X sensor
try:
    lox = VL53L0X(i2c)
    print("VL53L0X sensor initialized successfully.")
except Exception as e:
    print("Failed to initialize VL53L0X sensor:", e)
    exit(1)

# Start continuous ranging
print("Starting continuous ranging...")
lox.start_continuous()

try:
    while True:
        # Check if a range measurement is available
        distance = lox.range
        print(f"Distance in mm: {distance}")
        time.sleep(0.1)  # Delay to avoid flooding the output
except KeyboardInterrupt:
    print("Stopping continuous ranging...")
    lox.stop_continuous()
    print("Program terminated.")