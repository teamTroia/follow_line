#include "Encoder.h"

Encoder::Encoder(){}

void Encoder::initEncoder(){
    pinMode(ENC2A,INPUT);  
    pinMode(ENC2B,INPUT);  
    pinMode(ENC1A,INPUT);  
    pinMode(ENC1B,INPUT); 
}

double Encoder::getDistance(int pos){
    double dist = (2*PI*raio*pos)/freqEncoder;
    return dist;
}

void Encoder::keepPositon(){
    if(contPosicoes <= qtdTrechos){
        double pos = (getDistance(posDir)+getDistance(posEsq)/2.0);
        posicoes[contPosicoes] = pos;

        Serial.print("Posicao: ");
        Serial.println(posicoes[contPosicoes]);
        contPosicoes++;
    }
}

void Encoder::resetPosition(){
    posDir = 0;
    posEsq = 0;
}

void Encoder::readEncoderDir(){
    int a = digitalRead(ENC2A);
    int b = digitalRead(ENC2B);  
  
    if(b == a) posDir++;  
    else posDir--;

    //Serial.print("Posicao: ");
    //Serial.println(posDir);
}

void Encoder::readEncoderEsq(){
    int a = digitalRead(ENC1A);
    int b = digitalRead(ENC1B);  
  
    if(b != a) posEsq++;  
    else posEsq--;

    // Serial.print("Posicao: ");
    // Serial.println(posEsq);
    
}

void Encoder::readEncoder(){
    if(digitalRead(ENC2A) != estDir){
        estDir = !estDir;
        readEncoderDir();
    }

    if(digitalRead(ENC1A) != estEsq){
        estEsq = !estEsq;
        readEncoderEsq();
    }

}