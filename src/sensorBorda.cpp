#include "../include/types.h"
#include "../include/sensorBorda.h"

SensorBorda::SensorBorda() {

}

void SensorBorda::ler(bool valorBordaBool[]) {
    int valorBorda[2];
    pinMode(SBE, OUTPUT);
    pinMode(SBD, OUTPUT);
    digitalWrite(SBE, HIGH);
    digitalWrite(SBD, HIGH);
    float auxTemp = micros();
    while ((micros() - auxTemp) < 10); // Esperar por 10us
    // Sensor 0
   
    
    valorBorda[0] = 0;
    valorBorda[1] = 0;
    auxTemp = micros();
    pinMode(SBE, INPUT);
    pinMode(SBD, INPUT);
    while ((valorBorda[0] == 0 || valorBorda[1] == 0) && (micros() - auxTemp) < BORDA_RC_TIMEOUT) {
      if (!digitalRead(SBE) && valorBorda[0] == 0 ) {
        valorBorda[0] = micros() - auxTemp;
      }
      if (!digitalRead(SBD) && valorBorda[1] == 0 ) {
        valorBorda[1] = micros() - auxTemp;
      }
    }

    if (valorBorda[0] == 0) {
        valorBordaBool[0] = true;
    }else {
        valorBordaBool[0] = false;
    }
    if (valorBorda[1] == 0) { 
        valorBordaBool[1] = true;
    }else {
        valorBordaBool[1] = false;
    }
    if(DEBUGMODE) {
        Serial.print("Valor borda esquerda: ");
        Serial.println(valorBorda[0]);
        Serial.print("Valor borda direita: ");
        Serial.println(valorBorda[1]);
    }
    
}
