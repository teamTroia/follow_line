#include "./types.h"
#include "../include/motor.h"
//#define NSLEEP PB5

//Funções------------------------------------------
Motor::Motor () {
    imu = IMU();
    imu.init_mpu();
}

void Motor::motorInit() {
  
  pinMode (MAIN1, OUTPUT); // As variáveis IN são para acionamento da ponte-h, logo são saídas.(PWM).
  pinMode (MAIN2, OUTPUT);
  pinMode (MBIN1, OUTPUT);
  pinMode (MBIN2, OUTPUT);

  analogWrite(MAIN1, 0); // Todos os acionamentos são colocados em sinal baixo.
  analogWrite(MAIN2, 0);
  analogWrite(MBIN1, 0);
  analogWrite(MBIN2, 0);
}

void Motor::stop_MotorA() {
  analogWrite(MAIN1, 0);
  analogWrite(MAIN2, 0);
}
void Motor::stop_MotorB() {
  analogWrite(MBIN1, 0);
  analogWrite(MBIN2, 0);
}

void Motor::stop_Motor() {
  stop_MotorA();
  stop_MotorB();
}

void Motor::set_MotorA(int vel) {
  if (vel > 0) {
    analogWrite(MAIN2, 0);
    
    if (vel > 255) vel = 255;
    analogWrite(MAIN1, vel);
  } else  {
    analogWrite(MAIN1, 0);
    vel = -vel;

    if (vel > 255) vel = 255;
    analogWrite(MAIN2, vel);
  }
}
void Motor::set_MotorB(int vel) {
  if (vel > 0) {
    analogWrite(MBIN2, 0);

    if (vel > 255)
      vel = 255;
    analogWrite(MBIN1, vel);
  } else  {
    analogWrite(MBIN1, 0);
    vel = -vel;
    
    if (vel > 255) vel = 255;
    analogWrite(MBIN2, vel);
  }
}

void Motor::motorSetVel(int vel1, int vel2) {
  set_MotorA(vel1);
  //Serial.print(vel1);
  set_MotorB(vel2);
  //Serial.print(vel2);
}

float Motor::pid(float target, float atual) {
    float kp = 65;
    float kd = 0.05;
    // float ki = 0;
    
    float error = target - atual;
    float dt = (error - lastError) * kd;
    lastError = error;
    float output = error * kp + dt;
    return output;
}

void Motor::motorsControl(float linear, float angular) {

    angular = pid(angular, imu.readAngularSpeed());

    angular = angular > 100 ? 100 : angular;
    float Vel_R = linear - angular;  // ao somar o angular com linear em cada motor // conseguimos a ideia de direcao do robo
    float Vel_L = linear + angular;

    Vel_L = abs(Vel_L) < 10 ? 0 : Vel_L;
    Vel_R = abs(Vel_R) < 10 ? 0 : Vel_R;
    motorSetVel(Vel_L, Vel_R);
}