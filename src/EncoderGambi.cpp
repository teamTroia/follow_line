#include "EncoderGambi.h"

EncoderGambi::EncoderGambi(){}

bool EncoderGambi::verificaDistancia(uint64_t tempo){
    Serial.print("dist:");
    Serial.println(velocidade*tempo/1000);

    return velocidade*tempo/1000 >= distancias[aux]/2;
}

bool EncoderGambi::alteraVelocidade(uint64_t tempo){
    if(verificaDistancia(tempo)){
        aux++; 
        return 1;   
    }else{
        return 0;
    }
}

int EncoderGambi::retornaDistancia(uint64_t tempo){
    // Serial.print("dist:");
    // Serial.println(velocidade*tempo/1000);
    return velocidade*tempo/1000.0;
}
