#include <Wire.h>

class I2CSlave{
  private :
    union floatToBytes {
      byte valueBuffer[4];
      float valueReading;
    } converter;

  public :
    void setup();

    void receiveEvent(int byteCount);

    // Function that executes whenever data is requested by master
    void requestEvent();

    float getReadValue();
  };