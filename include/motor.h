#ifndef motor_H
#define motor_H

#include "./types.h"

class Motor {
private:

public:
    Motor();
    void init();
    void stopA();
    void stopB();
    void stopAll();
    void setA(uint16 vel);
    void setB(uint16 vel);
    void setAll(uint16 velA, uint16 velB);
};

#endif