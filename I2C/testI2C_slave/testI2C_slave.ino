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

union floatToBytes {
  byte valueBuffer[4];
  float valueReading;
} converter;

// void receiveEvent(int byteCount){
//   while(Wire.available()){
//     converter.valueBuffer[index] = Wire.read();
//     Serial.println(converter.valueBuffer[index]);
//     index++;
//   }
//   index = 0;
//   Serial.print("The number is: ");
//   Serial.println(converter.valueReading);
// }

void receiveEvent(int byteCount){
  for(uint8_t index = 0; index<byteCount; index++){
      converter.valueBuffer[index] = Wire.read();
      Serial.print(index);
      Serial.println(converter.valueBuffer[index]);
  }
  
  flag = true;
}

// Function that executes whenever data is requested by master
void requestEvent(){
  const char *response = "Hello Master!";
  Wire.write(response);
}




