#ifndef PID_H
#define PID_H


#include "./types.h"
#include "./sensores.h"
#include "./motor.h"

class PID {
    private:
        Motor motores;
        Sensores sensores;
    public:
        PID();

        int calcula_PID();
        void controla_motor(); 
};
#endif

