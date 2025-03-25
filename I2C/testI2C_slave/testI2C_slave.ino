
#include <Wire.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(8);                  // Join I2C bus with address #8
  Wire.onReceive(receiveEvent); // Register receive event
  Wire.onRequest(requestEvent); // Register request event
}

void loop() {
  // put your main code here, to run repeatedly:

}

void receiveEvent(int howMany){
  volatile int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}

// Function that executes whenever data is requested by master
void requestEvent(){
  const char *response = "Hello Master!";
  Wire.write(response);
}




