#include "./PID.h"

//---------------------------------------Parâmetros PID---------------------------------------
#define vel_motor_esquerdo 50
#define vel_motor_direito 50 
int Kp = 35;
int Kd = 35;
int Ki = 35;


PID::PID(){
    motores = Motor();
    sensores = Sensores();
}


int PID::calcula_PID(){
    /* 
    PID negativo = motor da esquerda mais rápido, ou seja, robô tendendo à direita
    PID positivo = motor da direita mais rápido, ou seja, robô tendendo à esquerda
    */

    int PID = 0, I = 0, P = 0, D = 0, erro = 0, erro_anterior = 0;

    erro = sensores.calcula_erro();

    if(erro == 0){
        I = 0;
    } 
    P = erro;       //Cálculo do erro proporcional (que é o mesmo que o erro no momento)
    I = I + erro;   //Cálculo do erro integrativo

    if(I > 255){  //Estabelece os limites do PWM
        I = 255;
    }
    else if(I < -255){
        I = -255;
    }

    D = erro - erro_anterior; //Cálculo do erro derivativo
    erro_anterior = erro;
    PID = (Kp*P) + (Ki*I) + (Kd*D);

    return PID;
}

void PID::controla_motor(){

    int PID, vel_esq, vel_dir;
    PID = calcula_PID();

    if(PID > 0){
        vel_dir = vel_motor_direito - PID;
        vel_esq = vel_motor_esquerdo;
    } else{
        vel_dir = vel_motor_direito;
        vel_esq = vel_motor_esquerdo + PID;
    }
    motores.motorSetVel(vel_esq,vel_dir);
}
