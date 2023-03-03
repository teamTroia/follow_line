#include "./motor.h"

Motor::Motor(){}


void Motor::motorInit() {
  pinMode (MAIN1, OUTPUT); 
  pinMode (MAIN2, OUTPUT);
  pinMode (MBIN1, OUTPUT);
  pinMode (MBIN2, OUTPUT);

  digitalWrite(MAIN1, 0); 
  digitalWrite(MAIN2, 0);
  digitalWrite(MBIN1, 0);
  digitalWrite(MBIN2, 0);
}

void Motor::stop_Motor() {
  digitalWrite(MAIN1, 0);
  digitalWrite(MAIN2, 0);
  digitalWrite(MBIN1, 0);
  digitalWrite(MBIN2, 0);
}

/*
void Motor::set_MotorA(int vel) {
  if (vel > 0) {
    analogWrite(MAIN2, 0);
    vel  = vel * MAXVELA *  0.0255;
    if (vel > 255) vel = 255;
    analogWrite(MAIN1, vel);
  } else  {
    analogWrite(MAIN1, 0);
    vel = -vel * MAXVELA * 0.0255;
    if (vel > 255) vel = 255;
    analogWrite(MAIN2, vel);
  }
}
void Motor::set_MotorB(int vel) {
  if (vel > 0) {
    analogWrite(MBIN2, 0);
    vel  = vel * MAXVELB *  0.0255;
    if (vel > 255)
      vel = 255;
    analogWrite(MBIN1, vel);
  } else  {
    analogWrite(MBIN1, 0);
    vel = -vel * MAXVELB * 0.0255;
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

*/