#include "Sensores.h"

Sensores::Sensores(){}

void Sensores::initSensors(){
    sensores.setTypeAnalog(); //Define os sensores frontais como analógicos
    borda.setTypeRC(); //Define os sensores de borda como digitais 
}

void Sensores::setSensorsPins(){
    borda.setSensorPins((const uint8_t[]){PB10, PA8},qtd_borda); //Definição dos pinos dos sensores de borda (direita esquerda, nessa ordem)
    sensores.setSensorPins((const uint8_t[]){PA15, PA6, PA5, PA4, PA3, PA0, PB1, PA9}, qtd_sensores); //Definição dos pinos dos sensores frontais, da ESQUERDA PARA DIREITA (lembra que eu observei a Top Layer, ou seja, PA0 = S1)
}

void Sensores::calibrateSensors(){
    for (int i = 0; i < 2; i++){ //Aqui tem que ver quantas vezes ele tem q passar pelo processo de calibração
        sensores.calibrate();
    
        digitalWrite(LED1, LOW); //Pisca os leds
        digitalWrite(LED2, LOW); //Pisca os leds
        delay(200);
        digitalWrite(LED1, HIGH); //Pisca os leds
        digitalWrite(LED2, HIGH); //Pisca os leds
        delay(200);
    }
        borda.calibrate();
    calibrado = 1; //Indica que o robô já foi calibrado e pode sair do loop de calibração
}

bool Sensores::getCalibrado(){
    return calibrado;
}

void Sensores::readSensors(){
    sensores.readCalibrated(valores_sensor);
    borda.read(valores_borda);

    //Caso seja necessário averiguar os valores lidos pelos sensores, descomente essa parte abaixo:
/*        
        for (uint8_t i = 0; i < qtd_sensores; i++){
            Serial.print("Sensor");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.println(valores_sensor[i]);
        }

        for (uint8_t i = 0; i < 2; i++){
            Serial.print("Sensor borda ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_borda[i]);
        }
        */
}

void Sensores::setCalibrado(bool calibrado){
    this->calibrado = calibrado;
}