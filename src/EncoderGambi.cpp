#include <EncoderGambi.h>

EncoderGambi::EncoderGambi(){}

bool EncoderGambi::verificaDistancia(uint64_t tempo){
    return velocidade*tempo/1000 >= distancias[aux];
}

bool EncoderGambi::alteraVelocidade(uint64_t tempo){
    if(verificaDistancia(tempo)){
        aux++; 
        return 1;   
    }else{
        return 0;
    }
}