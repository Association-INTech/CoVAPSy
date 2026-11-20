#ifndef _CONFIG_H
#define _CONFIG_H

// TODO: organize constants and parameters

#define SERIAL_RATE 115200

#define I2C_ADDR 0x8

// Quand on compile pour la première voiture (voiture lidar) il faut définir VOITURE1
// Pour la deuxième (voiture camera, voiture awake) il faut définir VOITURE2
// #define VOITURE1
#define VOITURE2

#ifndef VOITURE1
  #ifndef VOITURE2
    #error Aucune voiture n'est spécifiée
  #endif
#endif

#ifdef VOITURE1
  #ifdef VOITURE2
    #error Les deux voitures sont spécifiées
  #endif
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

#define SOUND_ON true

const int sensorPin_Lipo = A2; // select the input pin for the battery sensor
const int sensorPin_NiMh = A3; // select the input pin for the battery sensor

// constantes utiles
//
const float r1_LiPo = 560;  // resistance of the first resistor
const float r1_NiMh = 560;  // resistance of the first resistor
#if defined(VOITURE1)
const float r2_LiPo = 1500; // resistance of the second resistor
#elif defined(VOITURE2)
const float r2_LiPo = 1000;
#endif
const float r2_NiMh = 1000; // resistance of the second resistor

const int nb_trous = 16 * 2; // nb de trous dans la roue qui tourne devant la fourche optique
const int distanceUnTour = 79; // distance parcourue par la voiture apres un tour de roue en mm

#define V_REF 5 // Le voltage de réference pour les lectures analogues de la arduino nano est 5V
#define ANALOG_MAX 1023.0 // La valeur analogique maximale qui peut être lue est 1023.0

// config pwm
#if defined(VOITURE1)
  const float dir_max_pwm = 2501; // direction maximal physique en pwm 2501
  const float dir_min_pwm = 1261; // direction minimal physique en pwm 1261
#elif defined(VOITURE2)
  const float dir_max_pwm = 1100; // direction maximal physique en pwm 1100
  const float dir_min_pwm = 1900; // direction minimal physique en pwm 1900
#endif

const float dir_max = 30;      // direction maximale en valeur absolue recue entre une roue tourné a fond et une roue droite en degré avant conversion (via map) 

// Source: https://arduinogetstarted.com/reference/library/servo-writemicroseconds
const int motor_middle = 1500;
const int motor_stop = 1500;

#endif
