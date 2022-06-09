#ifndef types_H
#define types_H

#include <Arduino.h>
#define BORDA_RC_TIMEOUT 900
#define pivo 3000
#define DEBUGMODE true

//Pinout seguindo a placa 2020

#define LED1 PB12
#define LED2 PB0
#define LED3 PA9
#define BOT1 PB3
#define BOT2 PB4
#define BOT3 PB12
#define MAPIN1 PB8
#define MAPIN2 PB14
#define MBPIN1 PB9
#define MBPIN2 PB15
#define ENA1 PA10
#define ENA2 PA11
#define ENB1 PB5
#define ENB2 PB11
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