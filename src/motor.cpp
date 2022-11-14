#include "./types.h"
#include "../include/motor.h"
//#define NSLEEP PB5

//Funções------------------------------------------
Motor::Motor () {

}

void Motor::motorInit() {
  //Primeiro jogar o Nsleep p alto, isso ira desligar a ponte H
 // pinMode (NSLEEP, OUTPUT);
  //digitalWrite(NSLEEP, HIGH);
  //Jogar os dois pinos de entrada do motor A para nivel baixo
  pinMode (MAIN1, OUTPUT);
  pinMode (MAIN2, OUTPUT);
  //digitalWrite(MAIN1, LOW);
  //digitalWrite(MAIN2, LOW);

  //Agora Liga a ponte H e esta pronta para usar :p
  //digitalWrite(NSLEEP, LOW);

  pinMode (MAIN1, OUTPUT); // As variáveis IN são para acionamento do TLE, logo são saídas.(OUTPUT).
  pinMode (MAIN2, OUTPUT);
  pinMode (MBIN1, OUTPUT);
  pinMode (MBIN2, OUTPUT);

  analogWrite(MAIN1, 0); // Todos os acionamentos são colocados em sinal baixo.
  analogWrite(MAIN2, 0);
  analogWrite(MBIN1, 0);
  analogWrite(MBIN2, 0);

  //Serial.println("Motores inicializados!");
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
