#include "Sensores.h"

Sensores::Sensores(){}

void Sensores::initSensors(){
    for (int i = 0; i < qtd_sensores; i++)
        pinMode(sensorPins[i],INPUT_ANALOG);
    
    for (int i = 0; i < qtd_borda; i++)
        pinMode(bordaPins[i],INPUT);
}

void Sensores::calibrateSensors(){
    for (int i = 0; i < 5; i++){ 

        uint16_t sensorValues[8];
        uint16_t maxSensorValues[8];
        uint16_t minSensorValues[8];

        minimum = (uint16_t *)realloc(minimum, sizeof(uint16_t) * qtd_sensores);
        maximum = (uint16_t *)realloc(maximum, sizeof(uint16_t) * qtd_sensores);

        for (int i = 0; i < qtd_sensores; i++){
            minimum[i] = 0;
            maximum[i] = 2500;
        }

        for (uint8_t j = 0; j < 10; j++){
            readSensors();

            for (uint8_t i = 0; i < qtd_sensores; i++){
                if ((j == 0) || (sensorValues[i] > maxSensorValues[i]))
                    maxSensorValues[i] = sensorValues[i];

                if ((j == 0) || (sensorValues[i] < minSensorValues[i]))
                    minSensorValues[i] = sensorValues[i];
            }
        }

        for (uint8_t i = 0; i < qtd_sensores; i++){
            if (minSensorValues[i] > maximum[i])
                maximum[i] = minSensorValues[i];

            if (maxSensorValues[i] < minimum[i])
                minimum[i] = maxSensorValues[i];
        }

        digitalWrite(LED1, LOW); //Pisca os leds
        delay(200);
        digitalWrite(LED1, HIGH); //Pisca os leds
        delay(200);
    }

    calibrado = 1; //Indica que o robô já foi calibrado e pode sair do loop de calibração
}

bool Sensores::getCalibrado(){
    return calibrado;
}

void Sensores::readSensors(){
    for (uint8_t j = 0; j < 4; j++){
        for (uint8_t i = 0; i < qtd_sensores; i ++)
          valores_sensor[i] += analogRead(sensorPins[i]);
      }

    for (uint8_t i = 0; i < qtd_sensores; i ++){
        valores_sensor[i] = (valores_sensor[i])/4;
    }   
}

void Sensores::readBorda(){
    for (uint8_t i = 0; i < qtd_borda; i ++){
        valores_borda[i] = 2500;
        pinMode(bordaPins[i], OUTPUT);
        digitalWrite(bordaPins[i], HIGH);
    }

    delayMicroseconds(10); // charge lines for 10 us

    uint32_t startTime = micros();
    uint16_t time = 0;

    for (uint8_t i = 0; i < qtd_borda; i ++)
      pinMode(bordaPins[i], INPUT);

    while (time < 2500){
      time = micros() - startTime;

      for (uint8_t i = 0; i < qtd_borda; i ++){
        if ((digitalRead(bordaPins[i]) == LOW) && (time < valores_borda[i]))
          valores_borda[i] = time;
      }
    }

    /*
    for (uint8_t i = 0; i < qtd_borda; i++){
            Serial.print("Sensor borda ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_borda[i]);
    }
    */
}

void Sensores::readCalibrated(){
    readSensors();

    for (uint8_t i = 0; i < qtd_sensores; i++){
        uint16_t calmin, calmax;

        calmax = maximum[i];
        calmin = minimum[i];

        uint16_t denominator = calmax - calmin;
        int16_t value = 0;

        if (denominator != 0)
            value = (((int32_t)valores_sensor[i]) - calmin) * 1000 / denominator;

        if (value < 0) { value = 0; }
        else if (value > 1000) { value = 1000; }

        valores_sensor[i] = value;

        for (uint8_t i = 0; i < qtd_sensores; i++){
            Serial.print("Sensor");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.println(valores_sensor[i]);
        }
    }
}

void Sensores::setCalibrado(bool calibrado){
    this->calibrado = calibrado;
}