#include <Servo.h>
#include <EnableInterrupt.h>
#include <Wire.h>


#define PIN_DIR 10
#define PIN_MOT 9
#define PIN_FOURCHE A0

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
float vitesse=0; //vitesse de la voiture calculé
int dir = 0; 
float dir_recue = 90.0; //initialisation de la direction au centre

float dir_max_pwm = 2501; //direction maximal physique en pwm 2501
float dir_min_pwm = 1261; //direction minimal physique en pwm 1261
float dir_max = 30;      //direction maximal en valeur abosule recue entre une roue tourné a fond et une roue droite en degré avant conversion (via map) 


//PID
float vieuxEcart=0;
float vieuxTemps=0; //variable utilisee pour mesurer le temps qui passe
float Kp=0.05; //correction prop
float Ki=0.02; //correction integrale
float Kd=0.; //correction derivee
float integral=0;//valeur de l'integrale dans le PID
float derivee=0; //valeur de la derivee dans le PID

//mesures
volatile int count=0; //variable utilisee pour compter le nombre de fronts montants/descendants
volatile int vieuxCount=0; //stocke l'ancienne valeur de count pour ensuite faire la difference
volatile byte state=LOW;
float mesures[3]; // tableau de mesures pour lisser

//I2C
union floatToBytes {
      byte valueBuffer[4];
      float valueReading;
    } converter;

//struct pour la fréquence d'éxécution des fonctions
struct Task {
  unsigned long period;
  unsigned long last;
  void (*run)();
};

enum DriveState {
  DRIVE_STOP,
  DRIVE_FORWARD,
  DRIVE_REV_ARM_1,  // neutre
  DRIVE_REV_ARM_2,  // reverse fort
  DRIVE_REV_ARM_3,  // retour neutre
  DRIVE_REVERSE
};

DriveState driveState = DRIVE_STOP;
unsigned long driveTimer = 0;

//Voltage
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

int out;
int marche_avant = 1; //on initie la marche avant au début (0 étant la marche arriérer)
unsigned long marche_arriere_time = 0;

unsigned long dernier_input = millis();
//direction millieu 1851
// tout a gaucge 1231
// tout a droite 2471

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
  count++;
}


float PID(float cons, float mes, float dt,float old_out) {

  if ( old_out <= 0 && cons > 0){     // pour pouvoir sauter directement dans la plage de pwm ou la roue bouge et une transition plus fluide entre marche arriere et avant 
    integral = 2500;                  // valeur experimentale
  }
  else if (old_out >= 0 && cons <0 ){ // pour pouvoir sauter directement dans la plage de pwm où la roue bouge et une transition plus fluide entre marche avant et arriere
    integral = -5000;                 // valeur experimentale
  }

  float adjustedMes = (cons < 0) ? -mes : mes; // Adjust the measured speed based on the sign of the desired speed
  float e = cons - adjustedMes;                // Calculate the error
  float P = Kp * e;                            // Proportional term
  integral = integral + e * dt;                // Integral term
  float I = Ki * integral;  
  derivee = (e - vieuxEcart) / dt;             // Derivative term
  vieuxEcart = e;
  float D = Kd * derivee;
  return P + I + D;
}


void calculateVoltage(){
  //read from the sensor
  // and convert the value to voltage
  voltage_LiPo = analogRead(sensorPin_Lipo);
  voltage_NiMh = analogRead(sensorPin_NiMh);
  voltage_LiPo = voltage_LiPo * (5.0 / 1023.0) * ((r1_LiPo + r2_LiPo) / r1_LiPo);
  voltage_NiMh = voltage_NiMh * (5.0 / 1023.0) * ((r1_NiMh + r2_NiMh) / r1_NiMh);
  //Serial.println(voltage_LiPo);
  //Serial.println(voltage_NiMh);
}

void updateDirection(){
  // Update direction
  dir = map(dir_recue,-dir_max,dir_max,dir_min_pwm,dir_max_pwm); // remape en degré
  direction.writeMicroseconds(dir);
}

void updateSpeed() {
  unsigned long now = millis();
  float dt = (now - vieuxTemps) * 0.001f;
  if (dt <= 0) dt = 0.001f;

  vitesse = getMeanSpeed(now - vieuxTemps);

  switch (driveState) {

    case DRIVE_STOP:
      moteur.writeMicroseconds(1500);
      integral = 0;
      out = 0;

      if (Vcons > 0) {
        driveState = DRIVE_FORWARD;
      }
      else if (Vcons < 0) {
        driveState = DRIVE_REV_ARM_1;
        driveTimer = now;
      }
      break;

    case DRIVE_FORWARD:
      out = PID(Vcons, vitesse, dt, out);
      moteur.writeMicroseconds(constrain(1500 + out, 1500, 2000));

      if (Vcons == 0) {
        driveState = DRIVE_STOP;
      }
      else if (Vcons < 0) {
        driveState = DRIVE_REV_ARM_1;
        driveTimer = now;
      }
      break;

    case DRIVE_REV_ARM_1:   // neutre court
      moteur.writeMicroseconds(1500);
      if (now - driveTimer >= 10) {
        driveState = DRIVE_REV_ARM_2;
        driveTimer = now;
      }
      break;

    case DRIVE_REV_ARM_2:   // reverse fort (armement ESC)
      moteur.writeMicroseconds(1000);
      if (now - driveTimer >= 150) {
        driveState = DRIVE_REV_ARM_3;
        driveTimer = now;
      }
      break;

    case DRIVE_REV_ARM_3:   // retour neutre
      moteur.writeMicroseconds(1500);
      if (now - driveTimer >= 10) {
        driveState = DRIVE_REVERSE;   // 🔒 marche arrière ENGAGÉE
      }
      break;

    case DRIVE_REVERSE:
      out = PID(-Vcons, vitesse, dt, out);
      moteur.writeMicroseconds(constrain(1500 - out, 500, 1500));

      if (Vcons == 0){
        moteur.writeMicroseconds(1500);
        integral = 0;
        out = 0;
      }
      else if (Vcons > 0) {
        driveState = DRIVE_FORWARD;
      }
      break;
  }

  vieuxTemps = now;



  #if 0
     Serial.print("temps en marche arriere: ");
     Serial.print(millis()- marche_arriere_time);
     Serial.print(",integrale");
     Serial.print(integral);
     Serial.print(",const:");
     Serial.print(200);
     Serial.print(",Vcons:");
     Serial.print(Vcons);
     Serial.print(",Vitesse:");
     Serial.print(vitesse);
     Serial.print(",Directino en degré:");
     Serial.print(dir_recue);
     Serial.print(",out2:");
     Serial.println(out);
  #endif
}

void receiveEvent(int byteCount){
  // Ignorer le premier octet "commande" du Raspberry
  if (Wire.available()) Wire.read(); // skip cmd byte

  if (byteCount >= 9) { // 1 cmd + 8 data

    dernier_input = millis();

    byte buffer[8];
    for (int i = 0; i < 8 && Wire.available(); i++) {
      buffer[i] = Wire.read();
    }

    float* vals = (float*)buffer;
    Vcons = vals[0];  // reçue en milimetre par secondes
    dir_recue = vals[1];  //reçue en degré. 
  } else {
    while (Wire.available()) Wire.read(); // vide le buffer
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(pinMoteur,OUTPUT);
  moteur.attach(pinMoteur,0,2000);

  pinMode(pinDirection,OUTPUT);
  direction.attach(pinDirection);
  
  pinMode(pinFourche,INPUT_PULLUP);
  enableInterrupt(PIN_FOURCHE, blink, CHANGE); //on regarde a chaque fois que le signal de la fourche change (Montants et Descendants)
  moteur.writeMicroseconds(1500);
  delay(2000);
  moteur.writeMicroseconds(1590);

  Wire.begin(8);                  // Join I2C bus with address #8
  Wire.onReceive(receiveEvent);   // Register receive event
  Wire.onRequest(requestEvent);   // Register request event
  pinMode(13,OUTPUT);

  delay(10);
  Serial.print("init");
}


Task tasks[] = {
  {5,   0, updateDirection}, // 200 Hz
  {20,  0, updateSpeed},     // 50 Hz
  {500, 0, calculateVoltage}   // 2 Hz
};

void loop() {
  unsigned long now = millis();

  if(now - dernier_input < 150){ // on vérifie si on a recue une commande dans les 100 dernière millisecondes sinon on arrete
    for (auto &t : tasks) {
      if (now - t.last >= t.period) {
        t.last = now;
        t.run();
      }
    }
  }
  else {
    moteur.writeMicroseconds(1500);
    dir = map(0,-dir_max,dir_max,dir_min_pwm,dir_max_pwm);
    direction.writeMicroseconds(dir);
  }
}


void requestEvent(){
  const int numFloats = 3; // Number of floats to send
  float data[numFloats] = {voltage_LiPo, voltage_NiMh, vitesse}; // Example float values to send
  byte* dataBytes = (byte*)data;
  Wire.write(dataBytes, sizeof(data));
}