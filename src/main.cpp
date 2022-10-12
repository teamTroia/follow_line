#include <Arduino.h>
 #include "../include/types.h"
 #include "../include/motor.h"
 #include "../include/sensores.h"
 #include "../include/trechos.h"

float KP_R = 0.7,       // CORRIGE MAIS RÁPIDO MAS CAUSA INSTABILIDADE ------------ 0.6 ---- 0.7
      KI_R = 0.005,     // CORRIGE NO LONGO TEMPO --------------------------------- 0.01 --- 0.005
      KD_R = 0.4,       // CORRIGE MAIS RÁPIDO ------------------------------------ 0.015 -- 0.00 
      Vel_R = 0.55,     // -------------------------------------------------------- 0.05 --- 0.05
      Vel_erro_R = 0.5; // ----------------esse de curva--------------------------- 0.08 --- 0.15

float KP_c_aberta = 0.33,
      KI_c_aberta = 0.15,
      KD_c_aberta = 0.80,
      Vel_c_aberta = 0.005,
      Vel_erro_c_aberta = 0.05;

float KP_c_fechada = 0.33,
      KI_c_fechada = 0.15,
      KD_c_fechada = 0.80,
      Vel_c_fechada = 0.002,
      Vel_erro_c_fechada = 0.5;


//Variáveis----------------------------------------------------------
float U[2],       // [V, W] - Sinal de controle [mm/s] [rad/s]
      Uv[2],      // [vL vR] - Sinal de Controle em cada motor
      ErSen,      // - Erro no Sensor de Linha[Un]
      ErSen_0,    // - Erro no Sensor de Linha no instante anterior [Un]
      ErSenInt;   // - Integral do Erro no Sensor de Linha

int senCurva = 0,
    senCurvaCont = 0,
    senCurvaAnt = 0,
    CurvaTemp = 0;

int senStarStop = 0,
    senStarStopCont = 0,
    senStarStopAnt = 0,
    StartStopTemp = 0;

int Trecho = 0, StartStop = 0;

unsigned long int T_Sen_0, T_Sen_1, T_Bat, T_Parada, tempo;
float KPs, KIs, KDs, VELs, VELerro, dT_Sen;

float bateriaV;
#define LOWBAT_LEVEL 1.78
bool LOWBAT = false, parou = 0;

void verificaBateria();
Motor motor = Motor();
Sensores sensor = Sensores();

int Iniciado = 0;
int sensorBordaDig[2]; //Valor Lido dos sensores de borda
float* sensorArrayErro;

void setup() {
  sensorArrayErro = new float;
  /*afio_remap(AFIO_REMAP_TIM3_PARTIAL);// tem q add isso no código tbm, no caso da PB5
  afio_remap(AFIO_REMAP_TIM2_PARTIAL_2);//PB11*/
  motor.motorInit();
  pinMode(BOT1, INPUT_PULLDOWN);
  pinMode(BOT2, INPUT_PULLDOWN);
  pinMode(LED_L1, OUTPUT);
  pinMode(LED_L3, OUTPUT);
  pinMode(analogBat, INPUT_ANALOG);
  //pinMode(MOSFET, OUTPUT);
  //pinMode(BUZZER_PIN, OUTPUT);
  Serial3.begin(9600);
  sensor.sensorInit();

  ErSen = ErSenInt = 0;

  T_Sen_0 = T_Sen_1 = micros();

  Trecho = 0;
  StartStop = 0;
  U[0] = 0; // velocidade linear
  U[1] = 0; // velocidade angular
}

void loop() {
  tempo=millis();
  //bot2 parou de funfar, depois tem q apagar essA GAMBI

 /* if (digitalRead(BOT1)) {
    B++;
    }*/
  while ((!digitalRead(BOT2)) && (Iniciado == 0)) {
    
    digitalWrite(LED_L1, HIGH); //1
    delay(50);
    //digitalWrite(LED_L1, LOW);
    delay(50);
    digitalWrite(LED_L3, HIGH);
    Serial.println("GO");
    delay(50);
    //digitalWrite(LED_L3, LOW);

    
    parou = 0;

    T_Sen_0 = T_Sen_1 = micros();

    delay(100);
    digitalWrite(LED_L1, HIGH); //1
    digitalWrite(LED_L3, LOW);
    delay(50);
    digitalWrite(LED_L1, LOW); //1
    digitalWrite(LED_L3, HIGH);
    delay(50);
    digitalWrite(LED_L1, HIGH); //1
    digitalWrite(LED_L3, LOW);
    delay(50);
    digitalWrite(LED_L1, LOW); //1
    digitalWrite(LED_L3, HIGH);
    delay(50);

    T_Parada = millis();
    if (digitalRead(BOT1)) {       //condição calibração
      sensor.sensorCalibrate();
      Serial.println("calibrando...");
      Iniciado = 1;
    }
  } 
  sensor.sensorLer(sensorArrayErro, sensorBordaDig);

  senCurvaAnt = senCurva;
  if (sensorBordaDig[1] != senCurva) {
    senCurvaCont++;
    if (senCurvaCont >= 3) {
      senCurva = sensorBordaDig[1];
    }
  } else {
    senCurvaCont = 0;
  }

  senStarStopAnt = senStarStop;
  if (sensorBordaDig[0] != senStarStop) {
    senStarStopCont++;
    if (senStarStopCont >= 3) {
      senStarStop = sensorBordaDig[0];
    }
  } else {
    senStarStopCont = 0;
  }

  if (senCurva == 1 && senCurvaAnt == 0) {
    Trecho = Trecho + 1;
    ErSenInt = 0;
  }

  if (senStarStop == 1 && senStarStopAnt == 0) {
    StartStop = StartStop + 1;
  }
 
  if (senCurva == 1 || senStarStop == 1) {
    digitalWrite(LED_L1, HIGH);
  } else digitalWrite(LED_L1, LOW);

  // Início ---------------------------------------------------------------------------------------------------------------------------------------------------------
  if (Iniciado != 0 && parou == 0) { // Logo depois de apertar o botão de Start
    digitalWrite(LED_L3, HIGH);
    if (micros() - T_Sen_0 >= 2000) {
      ErSen_0 = ErSen;
      ErSen = *sensorArrayErro;

      if (millis() - T_Parada <= 20000) {
        StartStop = 0;
      }
      
      #warning ("TRECHOS FIXO PODE DIFICULTAR PID, VOLTAR O USO DA VARIAVEL TRECHO")
      switch (trechos_tipo[0]) {
        case CURVA_ABERTA: 
          KPs = KP_c_aberta;
          KIs = KI_c_aberta;//0.000001 * 256;
          KDs = KD_c_aberta;//400 * 256;
          VELs = Vel_c_aberta*0.06;
          VELerro = 0.7*Vel_erro_c_aberta;
          break;
        case CURVA_FECHADA:
          KPs = KP_c_fechada;
          KIs = KI_c_fechada;//0.000001 * 256;
          KDs = KD_c_fechada;//400 * 256;
          VELs = Vel_c_fechada*0.08;
          VELerro = Vel_erro_c_fechada;
          break;
        case RETA_NORMAL:
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R;
          VELerro = Vel_erro_R;;
          break;
        case RETA_CURTA:
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R*0.5;
          VELerro = Vel_erro_R;
          break;
        case RETA_LONGA:
        default:
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R;
          VELerro = Vel_erro_R;

          #warning ("ESSA CONTAGEM DE TEMPO COM MILLIS PARECE ERRADA") 
          if(millis() > 1000 ){
            KPs = KP_R;
            KIs = KI_R;//0.000001 * 256;
            KDs = KD_R;//400 * 256;
            VELs = Vel_R*0.5;
            VELerro = Vel_erro_R;
          }
          break;
      }

     //-------------------PID------------------------------------------------------------

      U[0] = VELs  - VELerro * (abs(ErSen)/3);
      if (U[0] < 0) U[0] = 0; 
      T_Sen_0 = T_Sen_1;
      T_Sen_1 = micros();
      dT_Sen = (T_Sen_1 - T_Sen_0) / 1000000.0;
          
      ErSenInt += ErSen * dT_Sen;

      if (ErSenInt > 100) 
        ErSenInt  = 100;
      else if (ErSenInt < -100) 
        ErSenInt  = -100;

      U[1] =  KPs * ErSen + KIs * ErSenInt + KDs * (ErSen - ErSen_0) / dT_Sen ; 

      ///  U[0] =0;

      //Uv[0] = U[0] - (dist_L / 4.0) * U[1];
      //Uv[1] = U[0] + (dist_L / 4.0) * U[1];
      Uv[0] = U[0] - U[1] / 2.0;
      Uv[1] = U[0] + U[1] / 2.0;

      if (Uv[0] > 1.0 ) Uv[0] = 1;
      else if (Uv[0] < -1 ) Uv[0] = -1;

      if (Uv[1] > 1 ) Uv[1] = 1;
      else if (Uv[1] < -1 ) Uv[1] = -1;

      // if (Trecho == 0) {
      //   motorSetVel(Uv[0] * 0.2 * 65535, Uv[1] * 0.2 * 65535);
      // }else
      motor.motorSetVel(Uv[0] * 800, Uv[1] * 800);
    }
  } else {
    Trecho = 0;
    motor.stop_Motor();
  }
}

#define BATMULT 4.25531915 
void verificaBateria() {
  static int lowBat = 0;
  static float bateriaVH[10], bateriaVHSum = 0 ;
  bateriaVHSum -= bateriaVH[0];
  for (int ii = 0; ii < 9; ii++) {
    bateriaVH[ii] = bateriaVH[ii + 1];
  }
  bateriaVH[9] =  map(analogRead(analogBat), 0, 4095, 0, 3300) * BATMULT / 1000.0;
  bateriaVHSum += bateriaVH[9];
  bateriaV = bateriaVHSum / 10.0;
  if (bateriaV < 1.78 && bateriaV > 1.6) {
    bateriaV = 0;
    LOWBAT = false;
  } else if (bateriaV < LOWBAT_LEVEL) {
    lowBat++;
    if (lowBat > 10) LOWBAT = true;
  } else {
    lowBat = 0;
    LOWBAT = false;
  }
}