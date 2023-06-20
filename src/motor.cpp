#include "Motor.h"

Motor::Motor() {}

void Motor::init_motor(){
    pinMode(MAIN1,OUTPUT);
    pinMode(MBIN1,OUTPUT);
    pinMode(MAIN2,OUTPUT);
    pinMode(MBIN2,OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
}

void Motor::stop_motor(){
    digitalWrite(MAIN1,0);
    digitalWrite(MBIN1,0);
    digitalWrite(MAIN2,0);
    digitalWrite(MBIN2,0);
    digitalWrite(PWMA,0);
    digitalWrite(PWMB,0);
}

void Motor::speed_motor(float valor_PID, int velocidade, uint8_t velocidade_maxima){
    int vel_esquerdo = velocidade + valor_PID;
    int vel_direito = velocidade - valor_PID;

    vel_esquerdo = constrain(vel_esquerdo,-velocidade_maxima,velocidade_maxima); //Limita o valor da velocidade a no mínimo 0 e no máximo 255
    vel_direito = constrain(vel_direito,-velocidade_maxima,velocidade_maxima); //Limita o valor da velocidade a no mínimo 0 e no máximo 255

/*
    Serial.print("Vel_esquerdo: ");
    Serial.println(vel_esquerdo);

    Serial.print("Vel_direito: ");
    Serial.println(vel_direito);
*/

    if(vel_esquerdo > 0){
        digitalWrite(MAIN2,LOW);
        analogWrite(PWMA,vel_esquerdo);
        digitalWrite(MAIN1,HIGH);
    }else if(vel_esquerdo == 0){
        digitalWrite(PWMA,HIGH);
        digitalWrite(MAIN1,HIGH);
        digitalWrite(MAIN2,HIGH);
    }else{
        vel_esquerdo = -vel_esquerdo;
        digitalWrite(MAIN1,LOW);
        digitalWrite(MAIN2,HIGH);
        analogWrite(PWMA,vel_esquerdo);
    }

    if(vel_direito > 0){
        digitalWrite(MBIN2,LOW);
        analogWrite(PWMB,vel_direito);
        digitalWrite(MBIN1,HIGH);
    }else if(vel_direito == 0){
        digitalWrite(PWMB,HIGH);
        digitalWrite(MBIN1,HIGH);
        digitalWrite(MBIN2,HIGH);
    }else{
        vel_direito = -vel_direito;
        digitalWrite(MBIN1,LOW);
        digitalWrite(MBIN2,HIGH);
        analogWrite(PWMB,vel_direito);
    }
}

bool Motor::getLigado(){
    return ligado;
}

void Motor::setLigado(bool ligado){
    this->ligado = ligado;
}