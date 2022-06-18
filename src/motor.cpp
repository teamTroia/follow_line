/*  Motor para DRV8848
*/

#include "../include/types.h"
#include "../include/motor.h"

/*
  Inicia o objeto motor, setando os pinos como saida PWM
  e foraçando uma parada
*/
Motor::Motor() {

}
void Motor::init() {
  
  pinMode (MAPIN1, PWM); // As variáveis IN são para acionamento do TLE, logo são saídas.(OUTPUT).
  pinMode (MAPIN2, PWM);
  pinMode (MBPIN1, PWM);
  pinMode (MBPIN2, PWM);


  if (DEBUGMODE) {
    pwmWrite(MAPIN1, 0);
    pwmWrite(MAPIN2, 0);

    pwmWrite(MBPIN1, 60000);
    pwmWrite(MBPIN2, 0);
    delay(1000);
    pwmWrite(MAPIN1, 0);
    pwmWrite(MAPIN2, 60000);

    pwmWrite(MBPIN1, 0);
    pwmWrite(MBPIN2, 0);
    delay(1000);
  }
  pwmWrite(MAPIN1, 0); // Todos os acionamentos são colocados em sinal baixo.
  pwmWrite(MBPIN1, 0);
  pwmWrite(MAPIN2, 0);
  pwmWrite(MBPIN2, 0);

  
}
/*
  Para o motor A
*/
void Motor::stopA() {
  pwmWrite(MAPIN1, 0);
  pwmWrite(MAPIN2, 0);
}

/*
  Para o motor B
*/
void Motor::stopB() {
  pwmWrite(MBPIN1, 0);
  pwmWrite(MBPIN2, 0);
}

/*
  Para os dois motores
*/
void Motor::stopAll() {
  Motor::stopA();
  Motor::stopB();
}

/*
  Seta velocidade do motor A
*/
void Motor::setA(uint16 vel) {
  if (vel > 0) {
    pwmWrite(MAPIN2, 0);
    vel  = vel * MAXVELA *  6.5535;
    if (vel > 65535) vel = 65535;
    pwmWrite(MAPIN1, vel);
  } else  {
    pwmWrite(MAPIN1, 0);
    vel = -vel * MAXVELA * 6.5535;
    if (vel > 65535) vel = 65535;
    pwmWrite(MAPIN2, vel);
  }
}

/*
  Seta velocidade do motor B
*/
void Motor::setB(uint16 vel) {
  if (vel > 0) {
    pwmWrite(MBPIN2, 0);
    vel  = vel * MAXVELB *  6.5535;
    if (vel > 65535)
      vel = 65535;
    pwmWrite(MBPIN1, vel);
  } else  {
    pwmWrite(MBPIN1, 0);
    vel = -vel * MAXVELB * 6.5535;
    if (vel > 65535) vel = 65535;
    pwmWrite(MBPIN2, vel);
  }
}

/*
  Seta velocidade dos motores
*/
void Motor::setAll(uint16 velA, uint16 velB) {
  Motor::setA(velA);
  Motor::setB(velB);
}
