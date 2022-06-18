#include "../include/funcsAux.h"
#include "../include/motor.h"

FuncsAux::FuncsAux () {

}

void FuncsAux::estadoLeds (bool estado) {
  if (estado) {
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,HIGH);
  } else {
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
  }
}

void FuncsAux::fim (Motor motores) {
  int cont = 0;
  while (true) {
    if (DEBUGMODE) Serial.println("ACABOOOOUUUU!!!");
    motores.stopAll();
    cont ++;
    estadoLeds((cont % 10) ? true : false);
  }
  return;
}