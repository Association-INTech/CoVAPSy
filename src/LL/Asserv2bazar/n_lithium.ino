#include <Wire.h>


const float r1_LiPo = 560;  // resistance of the first resistor
const float r1_NiMh = 560;  // resistance of the second resistor
const float r2_LiPo = 1500; // resistance of the second resistor
const float r2_NiMh = 1000; // resistance of the second resistor
float voltage_LiPo = 0;     // variable to store the value read
float voltage_NiMh = 0;     // variable to store the value read


void calculateVoltage(){
  //read from the sensor
  // and convert the value to voltage
  voltage_LiPo = analogRead(sensorPin_Lipo);
  voltage_NiMh = analogRead(sensorPin_NiMh);
  voltage_LiPo = voltage_LiPo * (5.0 / 1023.0) * ((r1_LiPo + r2_LiPo) / r1_LiPo);
  voltage_NiMh = voltage_NiMh * (5.0 / 1023.0) * ((r1_NiMh + r2_NiMh) / r1_NiMh);
  Serial.println(voltage_LiPo);
  Serial.println(voltage_NiMh);
}