/*  Motor para DRV8848
*/

//#Define-------------------------------------------
// Pinos dos motores A e B -> PWM IN1 IN2, STBY
#define MAIN1 PA10
#define MAIN2 PA11
#define MBIN1 PB7
#define MBIN2 PB6

#define MAXVELA 40
#define MAXVELB 40

//Headers-------------------------------------------
inline void motorInit(void) __attribute__((always_inline));                   //Inicializa Motor
inline void motorSetVel(int velA, int velB ) __attribute__((always_inline));  //seta velocidade -255a255 do motoa A e B
inline void motorStop(void) __attribute__((always_inline));                   //Trava motores

//Variáveis-----------------------------------------

//Funções------------------------------------------
void motorInit() {
  pinMode (MAIN1, PWM); // As variáveis IN são para acionamento do TLE, logo são saídas.(OUTPUT).
  pinMode (MAIN2, PWM);
  pinMode (MBIN1, PWM);
  pinMode (MBIN2, PWM);

  pwmWrite(MAIN1, 0); // Todos os acionamentos são colocados em sinal baixo.
  pwmWrite(MAIN2, 0);
  pwmWrite(MBIN1, 0);
  pwmWrite(MBIN2, 0);

  //Serial.println("Motores inicializados!");
}

void stop_MotorA() {
  pwmWrite(MAIN1, 0);
  pwmWrite(MAIN2, 0);
}
void stop_MotorB() {
  pwmWrite(MBIN1, 0);
  pwmWrite(MBIN2, 0);
}

void stop_Motor() {
  stop_MotorA();
  stop_MotorB();
}

void set_MotorA(int vel) {
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
void set_MotorB(int vel) {
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

void motorSetVel(int vel1, int vel2) {
  set_MotorA(vel1);
  //Serial.print(vel1);
  set_MotorB(vel2);
  //Serial.print(vel2);
}
