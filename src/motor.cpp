#include "./types.h"
#include "../include/motor.h"
//#define NSLEEP PB5

//Funções------------------------------------------
Motor::Motor () {

}

void Motor::motorInit() {
  
  pinMode (MAIN1, OUTPUT);
  pinMode (MAIN2, OUTPUT);
  digitalWrite(MAIN1, LOW);
  digitalWrite(MAIN2, LOW);

  pinMode (MAIN1, PWM); // As variáveis IN são para acionamento do TLE, logo são saídas.(OUTPUT).
  pinMode (MAIN2, PWM);
  pinMode (MBIN1, PWM);
  pinMode (MBIN2, PWM);

  pwmWrite(MAIN1, 0); // Todos os acionamentos são colocados em sinal baixo.
  pwmWrite(MAIN2, 0);
  pwmWrite(MBIN1, 0);
  pwmWrite(MBIN2, 0);

}

void Motor::stop_MotorA() {
  pwmWrite(MAIN1, 0);
  pwmWrite(MAIN2, 0);
}
void Motor::stop_MotorB() {
  pwmWrite(MBIN1, 0);
  pwmWrite(MBIN2, 0);
}

void Motor::stop_Motor() {
  stop_MotorA();
  stop_MotorB();
}

void Motor::set_MotorA(int vel) {
  if (vel > 0) {
    pwmWrite(MAIN2, 0);
    vel  = vel * MAXVELA *  6.5535;
    if (vel > 65535) vel = 65535;
    pwmWrite(MAIN1, vel);
  } else  {
    pwmWrite(MAIN1, 0);
    vel = -vel * MAXVELA * 6.5535;
    if (vel > 65535) vel = 65535;
    pwmWrite(MAIN2, vel);
  }
}
void Motor::set_MotorB(int vel) {
  if (vel > 0) {
    pwmWrite(MBIN2, 0);
    vel  = vel * MAXVELB *  6.5535;
    if (vel > 65535)
      vel = 65535;
    pwmWrite(MBIN1, vel);
  } else  {
    pwmWrite(MBIN1, 0);
    vel = -vel * MAXVELB * 6.5535;
    if (vel > 65535) vel = 65535;
    pwmWrite(MBIN2, vel);
  }
}

void Motor::motorSetVel(int vel1, int vel2) {
  set_MotorA(vel1);
  Serial.print(vel1);
  set_MotorB(vel2);
  Serial.print(vel2);
}
