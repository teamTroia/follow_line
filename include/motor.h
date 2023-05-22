#include "types.h"

class Motor {

    private:
        bool ligado = 0;
    public:
        Motor();
        void init_motor();
        void stop_motor();
        void speed_motor(float valor_PID, int velocidade, uint8_t velocidade_maxima);
        bool getLigado();
        void setLigado(bool ligado);
};