#ifndef funcsAux_H
#define funcsAux_H


#include "./types.h"
#include "../include/motor.h"

class FuncsAux {
    private:
        
    public:
        FuncsAux();
        void fim(Motor motores);
        void estadoLeds(bool estados);

};

#endif