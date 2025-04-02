#include <Servo.h>
#include <EnableInterrupt.h>
#include <Wire.h>


#define PIN_DIR 10
#define PIN_MOT 9
#define PIN_FOURCHE 14

Servo moteur;
Servo direction;

//declaration des broches des differents composants
const int pinMoteur=PIN_MOT;
const int pinDirection=PIN_DIR;
const int pinFourche=PIN_FOURCHE;

//constantes utiles
const int nb_trous=16*2; //nb de trous dans la roue qui tourne devant la fourche optique
const int distanceUnTour=79; //distance parcourue par la voiture apres un tour de roue en mm

//variables
char command;
float Vcons=0; 
float old_Vcons ;//consigne
float vitesse=0; //vitesse de la voiture

//PID
float vieuxEcart=0;
float vieuxTemps=0; //variable utilisee pour mesurer le temps qui passe
float Kp=0.05; //correction prop
float Ki=0.1; //correction integrale
float Kd=0.; //correction derivee
float integral=0;//valeur de l'integrale dans le PID
float derivee=0; //valeur de la derivee dans le PID

//mesures
volatile int count=0; //variable utilisee pour compter le nombre de fronts montants/descendants
volatile int vieuxCount=0; //stocke l'ancienne valeur de count pour ensuite faire la difference
volatile byte state=LOW;
float mesures[10]; // tableau de mesures pour lisser

////I2C
union floatToBytes {
      byte valueBuffer[4];
      float valueReading;
    } converter;

////Voltage
volatile bool dataReceived = false;
volatile char receivedData[32]; // Buffer to hold received data
volatile int receivedLength = 0;
        
const int sensorPin_Lipo = A2; // select the input pin for the battery sensor
const int sensorPin_NiMh = A3; // select the input pin for the battery sensor
        
const float r1_LiPo = 560;  // resistance of the first resistor
const float r1_NiMh = 560;  // resistance of the second resistor
const float r2_LiPo = 1500; // resistance of the second resistor
const float r2_NiMh = 1000; // resistance of the second resistor
float voltage_LiPo = 0;     // variable to store the value read
float voltage_NiMh = 0;     // variable to store the value read

float getMeanSpeed(float dt){
  int length = sizeof(mesures)/sizeof(mesures[0]);
  //ajout d'une nouvelle mesure et suppression de l'ancienne
  for (int i=length-1;i>0;i--){
      mesures[i]=mesures[i-1];
  }
  mesures[0] = getSpeed(dt);

  //Calcul d'une moyenne pour lisser les mesures qui sont trop dipersées sinon
  float sum=0;
  for (int i=0;i<length;i++){
    sum+=mesures[i];
  }

  //affichage debug
  #if 0
  for(int i=0;i<length;i++){
    Serial.print(mesures[i]);
    Serial.print(" , ");
  }
  Serial.println(sum/length);
  #endif

  return sum/length;
}

float getSpeed(float dt){  
  int N = count - vieuxCount; //nombre de fronts montant et descendands après chaque loop
  float V = ((float)N/(float)nb_trous)*distanceUnTour/(dt*1e-3); //16 increments -> 1 tour de la roue et 1 tour de roue = 79 mm 
  vieuxCount=count;
  vieuxTemps=millis();
  return V;
}
void blink(){ //on compte tous les fronts
  //Serial.print(count);
  count++;
}
float PID(float cons, float mes, float dt) {
  // Adjust the measured speed based on the sign of the desired speed
  float adjustedMes = (cons < 0) ? -mes : mes;

  // Calculate the error
  float e = cons - adjustedMes;

  // Proportional term
  float P = Kp * e;

  // Integral term
  integral = integral + e * dt;
  float I = Ki * integral;

  #if 0
  // Derivative term
  derivee = (e - vieuxEcart) / dt;
  vieuxEcart = e;
  float D = Kd * derivee;
  #endif

  return P + I;
}
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
void setup() {
  Serial.begin(115200);

  pinMode(pinMoteur,OUTPUT);
  moteur.attach(pinMoteur,0,2000);

  pinMode(pinDirection,OUTPUT);
  direction.attach(pinDirection,0,2000);
  
  pinMode(pinFourche,INPUT_PULLUP);
  enableInterrupt(PIN_FOURCHE, blink, CHANGE); //on regarde a chaque fois que le signal de la fourche change (Montants et Descendants)

  moteur.writeMicroseconds(1500);
  delay(2000);
  moteur.writeMicroseconds(1590);

  Wire.begin(8);                  // Join I2C bus with address #8
  Wire.onReceive(receiveEvent); // Register receive event
  Wire.onRequest(requestEvent); // Register request event
  pinMode(13,OUTPUT);

  delay(10);
  Serial.print("init");
}
void loop() {
  calculateVoltage();
  // Commandes pour debugger
  command=Serial.read();
  switch (command){ //pour regler les parametres
    case 'a':
    Vcons+=200;
    break;
    case 'z':
    Vcons-=200;
    break;
    case 's':
    Vcons=-1000;
    break;
    case 'd':
    Vcons=2000;
    break;
    case 'q':
    Vcons=0;
    break;
  }

  
  int deltaT = millis()-vieuxTemps; //temps qui est passé pendant un loop (en millisecondes)
  vitesse=getMeanSpeed(deltaT); // on recup la vitesse lissée
  
  int out;

  if (Vcons>=0){

    out = PID(Vcons,vitesse,float(deltaT)/1e3);
    moteur.writeMicroseconds(constrain(1500 + out,1500,2000));

  } else if ( Vcons<0 && old_Vcons>=0 ){
    
    out = PID(-8000,vitesse,float(deltaT)/1e3);
    moteur.writeMicroseconds(constrain(1500 + out,500,1500));
    delay(200);
    out = PID(0,vitesse,float(deltaT)/1e3);
    moteur.writeMicroseconds(constrain(1500 + out,1500,2000));
    delay(10);

  } else {

    out = PID(Vcons,vitesse,float(deltaT)/1e3);
    moteur.writeMicroseconds(constrain(1500 + out,500,1500));
  }

  old_Vcons = Vcons;

  //print debug
  #if 1
  Serial.print("");
  Serial.print(Vcons);
  Serial.print(", ");
  Serial.print(vitesse);
  Serial.print(", ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("out=");
  Serial.println(out);
  #endif
  delay(10);
}
void receiveEvent(int byteCount){
  for(uint8_t index = 0; index<byteCount; index++){
      converter.valueBuffer[index] = Wire.read();
  }
  Vcons = converter.valueReading;

}
void requestEvent(){
  const int numFloats = 2; // Number of floats to send
  float data[numFloats] = {voltage_LiPo, voltage_NiMh}; // Example float values to send
  byte* dataBytes = (byte*)data;

  Wire.write(dataBytes, sizeof(data));
}
