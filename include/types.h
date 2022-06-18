#ifndef types_H
#define types_H

#include <Arduino.h>
#define BORDA_RC_TIMEOUT 900
#define pivo 3000
#define DEBUGMODE true
#define SIMPLECONTROL true

//Velocidades
#define MAXVELA 100
#define MAXVELB 100

//Pinout seguindo a placa 2020

#define LED1 PB12
#define LED2 PB0
#define LED3 PA9
#define BOT1 PB3
#define BOT2 PB4
#define BOT3 PB12
#define ENA1 PB8
#define ENA2 PB14
#define ENB1 PB9
#define ENB2 PB15
#define MAPIN1 PA10 
#define MAPIN2 PA11
#define MBPIN1 PB7 //Era PB5
#define MBPIN2 PB6 //Era PB11
#define S1 PA7 
#define S2 PA6 
#define S3 PA5 
#define S4 PA4 
#define S5 PA3 
#define S6 PA2 
#define S7 PA1 
#define S8 PA0
#define SBE PA8
#define SBD PB10
#define pwm PB1
#define BAT PA12



#include <Arduino.h>

struct treshold {
  uint16 min;
  uint16 max;
};

#endif