#include <Arduino.h>

#define MAIN1 PA1
#define MAIN2 PB0
#define MBIN1 PA7 
#define MBIN2 PA2

#define MAXVELA 5
#define MAXVELB 5

#define INVERTER 0  //se quiser inverter os sensor colocar 1 se não 0
// Linha branca INVERTER = 0, linha preta INVERTER = 1

#define BORDA_RC 1
#define BORDA_RC_TIMEOUT 900
#define divisor 7 // divisor da faixa que determina o threshold

//MULTIPLEXDOR (SENSORES)
#define MULTIPLEX_SWITCH_A PA10
#define MULTIPLEX_SWITCH_B PA9
#define MULTIPLEX_SWITCH_C PA13
#define MULTIPLEX_COM PA0

#define LED_L1 PB12
#define LED_L2 PB1
#define LED_L3 PC13
#define BOT1 PB3
#define BOT2 PB4

#define dist_L 0.150 // distância entre rodas
#define analogBat PA12   // Pino do leitor de bateria



