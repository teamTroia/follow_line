#include "./types.h"

class Motor {

  private:
  /* data */
  public:

    Motor();  //seta velocidade -255a255 do motoa A e B 
    void motorSetVel(int vel1, int vel2);
    void set_MotorA(int vel);
    void set_MotorB(int vel);
    void stop_Motor();
    void stop_MotorA();
    void stop_MotorB();
    void motorInit();
};