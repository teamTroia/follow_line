#include "./types.h"
#include <QTRSensors.h>

QTRSensors sensores;
QTRSensors borda;
const uint8_t qtd_sensores = 6;
uint16_t valores_sensor[qtd_sensores];
uint16_t valores_borda[2];
bool calibrado = 0, btn_clicado = 0, ligado = 0; 

void setup(){
    pinMode(BTN1,INPUT_PULLDOWN);
    pinMode(BTN2,INPUT_PULLDOWN);
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    sensores.setTypeAnalog();
    borda.setTypeRC();

    borda.setSensorPins((const uint8_t[]){PB10, PA8},2);
    sensores.setSensorPins((const uint8_t[]){PB1, PA6, PA5, PA4, PA3, PA0}, qtd_sensores);
    Serial.begin(9600);
}

void calibracao();
void leitura();

void loop(){
    calibracao();
    leitura();
}

void calibracao(){
    while ((!digitalRead(BTN2)) && (calibrado == 0)) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);

    Serial.println("GO");

    delay(100);

    if(digitalRead(BTN1)){
        Serial.println("calibrando");

        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        delay(100);
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        
        for (int i = 0; i < 1000; i++){
            sensores.calibrate();
            //borda.calibrate();
        }
        calibrado = 1;
    }
  }
}

void leitura(){
    if(calibrado && (digitalRead(BTN2) or ligado)){
        sensores.read(valores_sensor);
        borda.read(valores_borda);
        for (uint8_t i = 0; i < qtd_sensores; i++){
            Serial.print("Sensor");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_sensor[i]);
        }
        for (uint8_t i = 0; i < 2; i++){
            Serial.print("Sensor borda ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_borda[i]);
        }
        delay(250);
        ligado = 1;
    }
}