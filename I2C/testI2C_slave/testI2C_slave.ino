
#include <Wire.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(8);                  // Join I2C bus with address #8
  Wire.onReceive(receiveEvent); // Register receive event
  Wire.onRequest(requestEvent); // Register request event
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
  delay(500);

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




