#include <Arduino.h>
#include <SoftwareSerial.h>

#include "../include/motor.h"
#include "../include/sensores.h"
#include "../include/PID.h"

#include "../include/types.h"
SoftwareSerial bluetooth(PB7, PB6);

// Projeto Apollo - Nelis e Pote


// PID --------------------------------------------------------------
char trechoTipo[] = {'R', 'A', 'C', 'F', 'C', 'F', 'C', 'F', 'C', 'F', 'F',
                     'F', 'R', 'F', 'R', 'F', 'R', 'F', 'R', 'F', 'R', 'F',
                     'R', 'A', 'R', 'A', 'R', 'A', 'R', 'A', 'R'};  // TREINO
// char trechoTipo[] = { 'R', 'F','C','F','C','F','C','A'
// ,'C','F','C','A','C','A','C','F','R','A','C','A','C','A','C','F','C','A','C','F','F','F','L'};
// //oficial dia 1 char trechoTipo[] = { 'R', 'A','C','F','C','F'
// ,'C','F','C','A','R'}; //oficial dia 1 char trechoTipo[] = { '1',
// '2','3','4','5','6'
// ,'7','8','9','10','11','12','13','14','15','16','17','18','19','20','21','22','23','24','25','26','27','28','29'};
// //OFICIAL

float KP_R = 0.6,  // CORRIGE MAIS RÁPIDO MAS CAUSA INSTABILIDADE --------------
                   // 0.6 ---- 0.7
    KI_R = 0.005,  // CORRIGE NO LONGO TEMPO ----------------------------------
                   // 0.01 --- 0.005
    KD_R = 0.1,    // CORRIGE MAIS RÁPIDO ------------------------------------
                   // 0.015 -- 0.00// 0.4
    Vel_R = 0.55,  // --------------------------------------------------------
                   // 0.05 --- 0.05
    Vel_erro_R =
        0.029;  // ----------------esse de
                // curva----------------------------------- 0.08 --- 0.15

float KP_c_aberta = 0.6, KI_c_aberta = 0.005, KD_c_aberta = 0.4,
      Vel_c_aberta = 0.2, Vel_erro_c_aberta = 0.02;

float KP_c_fechada = 0.6, KI_c_fechada = 0.005, KD_c_fechada = 0.4,
      Vel_c_fechada = 0.2, Vel_erro_c_fechada = 0.02;

// Variáveis----------------------------------------------------------
float U[2],    // [V, W] - Sinal de controle [mm/s] [rad/s]
    Uv[2],     // [vL vR] - Sinal de Controle em cada motor
    ErSen,     // - Erro no Sensor de Linha[Un]
    ErSen_0,   // - Erro no Sensor de Linha no instante anterior [Un]
    ErSenInt;  // - Integral do Erro no Sensor de Linha

int senCurva = 0, senCurvaCont = 0, senCurvaAnt = 0, CurvaTemp = 0;
int senStarStop = 0, senStarStopCont = 0, senStarStopAnt = 0, StartStopTemp = 0;

int Trecho = 0, StartStop = 0;

unsigned long int T_Sen_0, T_Sen_1, T_Bat, T_Parada, tempo;
float KPs, KIs, KDs, VELs, VELerro, dT_Sen;

float bateriaV;  // Tensão da Bateria
float LOWBAT_LEVEL = 1.78;
boolean LOWBAT = false,  // verdadeiro para indicar a bateria fraca
    parou = 0;

void verificaBateria();
Motor motor = Motor();
Sensores sensor = Sensores();
PID pid = PID();

int Iniciado = 0;


int sensorBordaDig[2];  // Valor Lido dos sensores de borda
float sensorArrayErro;
double tempoLeituraBorda = micros();
double tempoLeituraTrecho = micros();
int sSoma = 0;

void setup() {
    motor.motorInit();
    pinMode(BOT1, INPUT_PULLDOWN);
    pinMode(BOT2, INPUT_PULLDOWN);
    pinMode(LED_L1, OUTPUT);
    pinMode(LED_L3, OUTPUT);

    Serial.begin(9600);
    bluetooth.begin(9600);
    sensor.sensorInit();
}

void loop() {
    
    sensor.sensorLer(sensorArrayErro, sensorBordaDig, sSoma);
    sensor.calcula_erro();
    pid.calcula_PID();

    while ((!digitalRead(BOT2)) && (Iniciado == 0)) {
        for (int i =0; i<10; i++) {
            digitalWrite(LED_BUILTIN, ((i/2 == 0) ? HIGH : LOW));
            delay(50);
        }
        if (digitalRead(BOT1)) {  // condição calibração
            sensor.sensorCalibrate();
            Serial.println("calibrando...");
            Iniciado = 1;
        }
    }

    if (sensorBordaDig[1] && !sensorBordaDig[0]) {
        if ((micros() - tempoLeituraBorda) >= 900) {
            Trecho++;
            tempoLeituraTrecho = micros();
        }
    } else if (sensorBordaDig[1] && !sensorBordaDig[0]) {
        if ((micros() - tempoLeituraTrecho) >= 900) {
            Serial.print("Start Stop");
            StartStop++;
            tempoLeituraBorda = micros();
        }
    } else if (sensorBordaDig[1] && sensorBordaDig[0]) {
        
    }
    if (StartStop >= 2) {  // número de marcações para parar
        motor.stop_Motor();
        digitalWrite(LED_L3, HIGH);
    } else {
        //int velAng = sSoma * 2;
        //motor.motorsControl(-10, velAng);
        pid.controla_motor();
    }
}