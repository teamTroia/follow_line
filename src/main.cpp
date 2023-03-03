#include "./types.h"
#include <QTRSensors.h>

QTRSensors sensores;
const uint8_t qtd_sensores = 6;
uint16_t valores_sensor[qtd_sensores];
bool calibrado = 0, btn_clicado = 0, ligado = 0; 

void setup(){
    pinMode(BTN1,INPUT_PULLDOWN);
    pinMode(BTN2,INPUT_PULLDOWN);
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    sensores.setTypeAnalog();
    sensores.setSensorPins((const uint8_t[]){PB1, PA6, PA5, PA4, PA3, PA0}, qtd_sensores);
    Serial.begin(9600);
}

void calibracao();

void loop(){
    calibracao();
    

    if(calibrado && (digitalRead(BTN2) or ligado)){
        sensores.read(valores_sensor);
        for (uint8_t i = 0; i < qtd_sensores; i++){
            Serial.print("Sensor");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_sensor[i]);
        }
        delay(250);
        ligado = 1;
    }
}

void calibracao(){
    while ((!digitalRead(BTN2)) && (calibrado == 0)) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);

    Serial.println("GO");

    delay(100);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    delay(100);

    if(digitalRead(BTN1)){
        Serial.println("calibrando");
        for (int i = 0; i < 400; i++)
            sensores.calibrate();
        calibrado = 1;
    }
  }
}