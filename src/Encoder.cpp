#include "Encoder.h"

Encoder::Encoder(){}

void Encoder::readEncoderEsq(){
    int a = digitalRead(ENC1A);
    int b = digitalRead(ENC1B);  
  
    if(b > 0) Encoder::posEsq++;  
    else if(a > 0) Encoder::posEsq--;
}

void Encoder::readEncoderDir(){
    int a = digitalRead(ENC2A);
    int b = digitalRead(ENC2B);  
  
    if(b > 0) Encoder::posDir++;  
    else if(a > 0) Encoder::posDir--;
}

void Encoder::initEncoder(){
    pinMode(ENC2A,INPUT);  
    pinMode(ENC2B,INPUT);  
    pinMode(ENC1A,INPUT);  
    pinMode(ENC1B,INPUT); 
    attachInterrupt(digitalPinToInterrupt(ENC2A),readEncoderDir,RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2B),readEncoderDir,RISING);
    attachInterrupt(digitalPinToInterrupt(ENC1A),readEncoderEsq,RISING);
    attachInterrupt(digitalPinToInterrupt(ENC1B),readEncoderEsq,RISING);
}

double Encoder::getDistance(int pos){
    double dist = (2*PI*raio*pos)/freqEncoder;
    return dist;
}

void Encoder::keepPositon(){
    if(contPosicoes <= qtdTrechos){
        posicoesDir[contPosicoes] = getDistance(Encoder::posDir);
        Serial.println(getDistance(posDir));
        posicoesEsq[contPosicoes] = getDistance(Encoder::posEsq);
        contPosicoes++;
    }
}

void Encoder::resetPosition(){
    posDir = 0;
    posEsq = 0;
}