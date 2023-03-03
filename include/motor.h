#ifndef MOTOR_H
#define MOTOR_H

#include "./types.h"

class Motor {

  private:
    void set_MotorA(int vel);
    void set_MotorB(int vel);

  public:

    Motor(); 
    void motorSetVel(int vel1, int vel2);
    void stop_Motor();
    void motorInit();
};

#endif