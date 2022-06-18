#include "../include/types.h"
#include "../include/sensores.h"

/*
  Inica o objeto
  Inicia pinos dos sensores
    (Um monte de pinMode)
*/
Sensores::Sensores() {


}

/* 
  Calibração
    Recebe um pivo, ponto central, oque tiver acima é preto
    oque tiver abaixo é branco.
    Retorna um Struct com faixas de operação, 
    Struct composto por:
    {
      minimoBranco
      maximoBranco
      minimoPreto
      maximoPreto
    }
*/
void Sensores::init () {
  uint8 sensorPins[8] = {S1,S2,S3,S4,S5,S6,S7,S8};
  uint8 sensorBordaPins[2] = {SBD, SBE};
  uint8 pinosLedsBots[6] = {LED1, LED2, LED3, BOT1, BOT2, BOT3};
  
  for (unsigned int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT_ANALOG);
  }

  pinMode(sensorBordaPins[0], INPUT);
  pinMode(sensorBordaPins[1], INPUT);
  pinMode(pinosLedsBots[0], OUTPUT);
  pinMode(pinosLedsBots[1], OUTPUT);
  pinMode(pinosLedsBots[2], OUTPUT);
  pinMode(pinosLedsBots[3], INPUT_PULLDOWN);
  pinMode(pinosLedsBots[4], INPUT_PULLDOWN);
  pinMode(pinosLedsBots[5], INPUT_PULLDOWN);
}
/*

*/
void Sensores::sensorCalibrate(treshold tresholds[]) {
  uint8 sensorPins[8] = {S1,S2,S3,S4,S5,S6,S7,S8};
  uint16 valor;
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  for (int i = 0; i < 1000; i++) {
    if (DEBUGMODE) {
      Serial.println("Ta calibrando...");
    }
    if (i % 100 != 0) { //So para piscar os Leds durante a calibração
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
    }else {
      digitalWrite(LED1,HIGH);
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
    }

    for (int j = 0; j < 8; j++)
    {
      valor = analogRead(sensorPins[j]);
      Serial.println(valor);
      if(valor > pivo) { //Branco
        if (tresholds[j].max <= valor) {
          tresholds[j].max = valor;
        }
       }else { //Preto
        if (tresholds[j].min >= valor) {
          tresholds[j].min = valor;
        }
      }
    }
    delay(10);
  }

  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  digitalWrite(LED3,HIGH);
  
  if (DEBUGMODE) {
    for (int i = 0; i < 8; i++) {
      Serial.println(tresholds[i].max);
      Serial.println(tresholds[i].min);
    }
  }
}
