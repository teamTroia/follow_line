#ifndef MOTOR_H
#define MOTOR_H

#include "./types.h"
#include "../include/imuUtils.h"

class Motor {

  private:
    IMU imu;
    void set_MotorA(int vel);
    void set_MotorB(int vel);
    void stop_MotorA();
    void stop_MotorB();
    float lastError = 0;
    float pid(float target, float atual);
  public:

    Motor();  //seta velocidade -255a255 do motoa A e B 
    void motorSetVel(int vel1, int vel2);
    void stop_Motor();
    void motorInit();
    void motorsControl(float linear, float angular);
};

#endif