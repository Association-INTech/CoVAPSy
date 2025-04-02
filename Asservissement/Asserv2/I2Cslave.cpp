#include "VoltageReader.hpp"

class I2CSlave{
  static void setup() {
    // put your setup code here, to run once:
    Wire.begin(8);                  // Join I2C bus with address #8
    Wire.onReceive(receiveEvent); // Register receive event
    Wire.onRequest(requestEvent); // Register request event
    pinMode(13,OUTPUT);
  }

  static void receiveEvent(int byteCount){
    for(uint8_t index = 0; index<byteCount; index++){
        converter.valueBuffer[index] = Wire.read();
    }
  }

  // Function that executes whenever data is requested by master
  static void requestEvent(){
      TensionReader::request();
  }
  static float getReadValue(){
    return converter.valueReading;
  }


}