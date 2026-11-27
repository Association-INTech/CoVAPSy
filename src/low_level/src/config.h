#ifndef _CONFIG_H
#define _CONFIG_H

#define SERIAL_RATE 115200

#define I2C_ADDR 0x8

#define SOUND_ON true // Buzzer

#define NO_CONN_TIMEOUT 150 // Stop after 150ms without instruction

// Quand on compile pour la première voiture (voiture lidar) il faut définir VOITURE1
// Pour la deuxième (voiture camera, voiture awake) il faut définir VOITURE2
#define VOITURE1
// #define VOITURE2

#if !defined(VOITURE1) && !defined(VOITURE2)
  #error "Aucune voiture n'est spécifiée"
#endif

#if defined(VOITURE1) && defined(VOITURE2)
  #error "Les deux voitures sont spécifiées"
#endif

#if defined(VOITURE1)
  #warning "Code compilé pour la voiture 1"
#elif defined(VOITURE2)
  #warning "Code compilé pour la voiture 2"
#endif

// Pin definitions
#if defined(VOITURE1)
  #define PIN_DIR 10
#elif defined(VOITURE2)
  #define PIN_DIR 3
#endif

#define PIN_MOTEUR 9
#define PIN_FOURCHE A0
#define PIN_BUZZER 6

// Battery sensor pins
#define PIN_Lipo A2
#define PIN_NiMh A3

// config pwm
#if defined(VOITURE1)
  const float dir_max_pwm = 2501; // direction maximal physique en pwm 2501
  const float dir_min_pwm = 1261; // direction minimal physique en pwm 1261
#elif defined(VOITURE2)
  const float dir_max_pwm = 1100; // direction maximal physique en pwm 1100
  const float dir_min_pwm = 1900; // direction minimal physique en pwm 1900
#endif

const float motor_min_pwm = 0;
const float motor_max_pwm = 2000; // This limits the speed of the car

const float dir_max = 30; // direction maximale en valeur absolue recue entre une roue tournée a fond et une roue droite en degré 

// Hardware constants
// Read the low level docs paragraph about battery level for explanation
const float r1_NiMh = 560;
const float r1_LiPo = 560;
const float r2_NiMh = 1000;
#if defined(VOITURE1)
const float r2_LiPo = 1500;
#elif defined(VOITURE2)
const float r2_LiPo = 1000;
#endif

const int nb_trous = 16 * 2; // nb de trous dans la roue qui tourne devant la fourche optique
const int distanceUnTour = 79; // distance parcourue par la voiture apres un tour de roue en mm

#define V_REF 5.0 // Le voltage de réference pour les lectures analogues de la arduino nano est 5V
#define ANALOG_MAX 1023.0 // La valeur analogique maximale qui peut être lue est 1023.0

// Source: https://arduinogetstarted.com/reference/library/servo-writemicroseconds
// 1500 represents the middle for direction and stopping for propulsion
const int motor_middle = 1500;
const int motor_stop = 1500;

#endif
