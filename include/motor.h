#include "./types.h"


class Motor {
private:

public:
    Motor();
    void init();
    void stopA();
    void stopB();
    void stopAll();
    void setA(uint16 vel, uint8 MAXVELA);
    void setB(uint16 vel, uint8 MAXVELB);
    void setAll(uint16 velA, uint16 velB, uint8 MAXVELA, uint8 MAXVELB);
};