 #include <Arduino.h>
 #include "../include/types.h"
 #include "../include/motor.h"
 #include "../include/sensores.h"
 #include <SoftwareSerial.h>
 
 SoftwareSerial bluetooth(PB7,PB6);
 

 /*  Pandemia-2020-2021
    Follow line utilizando STM32F103C8T6 - TROIA
    por: Luara Linhares e Fidelis ihuuuu
*/

// PID --------------------------------------------------------------
char trechoTipo[] = { 'R', 'F', 'C', 'F', 'F', 'A', 'F', 'R', 'F', 'L', 'F', 'L', 'F', 'C', 'F', 'C', 'F', 'R', 'C', 'F', 'F', 'F', 'F', 'F', 'R', 'F', 'R', 'F', 'R', 'F', 'C', 'A', 'C', 'F', 'R','A','R','F','R', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'R', 'F', 'A', 'C'}; // pista RCX 2022

//{ 'R', 'A','C','F','C','F' ,'C','F','C','F','F','F','R','F','R','F','R','F','R','F','R','F','R','A','R','A','R','A','R','A','R'}; //TREINO
//char trechoTipo[] = { 'R', 'F','C','F','C','F','C','A' ,'C','F','C','A','C','A','C','F','R','A','C','A','C','A','C','F','C','A','C','F','F','F','L'}; //oficial dia 1
//char trechoTipo[] = { 'R', 'A','C','F','C','F' ,'C','F','C','A','R'}; //oficial dia 1
//char trechoTipo[] = { '1', '2','3','4','5','6' ,'7','8','9','10','11','12','13','14','15','16','17','18','19','20','21','22','23','24','25','26','27','28','29'}; //OFICIAL

float KP_R = 0.6, // CORRIGE MAIS RÁPIDO MAS CAUSA INSTABILIDADE -------------- 0.6 ---- 0.7
      KI_R = 0.006, // CORRIGE NO LONGO TEMPO ---------------------------------- 0.01 --- 0.005
      KD_R = 0.09, // 0.1CORRIGE MAIS RÁPIDO ------------------------------------ 0.015 -- 0.00// 0.4
      Vel_R = 0.6, // 0.6-------------------------------------------------------- 0.05 --- 0.05
      Vel_erro_R = 0.12; // 0.12 ----------------esse de curva----------------------------------- 0.08 --- 0.15

float KP_c_aberta = 0.6,
      KI_c_aberta = 0.006,
      KD_c_aberta = 0.09,
      Vel_c_aberta = 0.3,
      Vel_erro_c_aberta = 0.12;

float KP_c_fechada = 0.6,
      KI_c_fechada = 0.006,
      KD_c_fechada = 0.3,
      Vel_c_fechada = 0.6,
      Vel_erro_c_fechada = 0.12;


//Variáveis----------------------------------------------------------
float U[2],       // [V, W] - Sinal de controle [mm/s] [rad/s]
      Uv[2],       // [vL vR] - Sinal de Controle em cada motor
      ErSen,      // - Erro no Sensor de Linha[Un]
      ErSen_0,    // - Erro no Sensor de Linha no instante anterior [Un]
      ErSenInt;   // - Integral do Erro no Sensor de Linha

      //APAGA DEPOIS
      int B;

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

float bateriaV;  //Tensão da Bateria
float LOWBAT_LEVEL = 1.78;
boolean LOWBAT = false, // verdadeiro para indicar a bateria fraca
        parou = 0;

void verificaBateria();
Motor motor = Motor();
Sensores sensor = Sensores();

int Iniciado = 0;
int sensorBordaDig[2]; //Valor Lido dos sensores de borda
float sensorArrayErro;

void setup() {

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
  bluetooth.begin(9600);
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
  // while (true)
  //   {
  //     Serial.println(digitalRead(BOT1));
  //     Serial.println(digitalRead(BOT2));
  //     delay(3000);
  //   } 
  char dadoBluetooth;
  while ((!digitalRead(BOT2)) && (!Iniciado)) {

    
    parou = 0;
    T_Sen_0 = T_Sen_1 = micros();
    T_Parada = millis();
    Serial.println("Esperando inicio");
    bluetooth.println("Esperando incio");
    if(bluetooth.available()) {
      Serial.println("Bluetooth ok");
    } else {
      Serial.println("Bluetooh foi de comes");
    }
    dadoBluetooth = bluetooth.read();

    if(dadoBluetooth == 'A') {
      
      bluetooth.println("Editando Valores de Curva Aberta");
      Vel_erro_c_aberta = bluetooth.read();
      while (Vel_erro_c_aberta == -1) {
        Vel_erro_c_aberta = bluetooth.read();
      }
      bluetooth.print("Novo valor: ");
      bluetooth.println(Vel_erro_c_aberta);
    } else if (dadoBluetooth == 'F') {
      bluetooth.println("Editando Valores de Curva fechada");
      Vel_erro_c_fechada = bluetooth.read();
      while (Vel_erro_c_fechada == -1) {
        Vel_erro_c_fechada = bluetooth.read();
      }
      bluetooth.print("Novo valor: ");
      bluetooth.println(Vel_erro_c_fechada);
    } else if (dadoBluetooth == 'Q') {
      bluetooth.println("Quit from config mode");
    }
    

    if (digitalRead(BOT1)) {       //condição calibração
      sensor.sensorCalibrate();
      Serial.println("calibrando...");
    }
  }
  Iniciado = 1;
  sensor.sensorLer(sensorArrayErro, sensorBordaDig);
  
  Serial3.print(senStarStop); Serial3.print(" ,"); 
  Serial3.println(senCurva);
  
  // Sensores de Borda ----------------------------------------------
  
//sensor esquerdo apos detectar 2 marcas brancas, faz o robô parar
  
  senCurvaAnt = senCurva;
  if (sensorBordaDig[1] != senCurva) {
    senCurvaCont++;
    if (senCurvaCont >= 3) { //só deixa o sensor mudar o estado depois de detectar o preto ou o branco 2 vezes
      senCurva = sensorBordaDig[1];
    }
  } else {
    senCurvaCont = 0;
  }

  senStarStopAnt = senStarStop;
  if (sensorBordaDig[0] != senStarStop) {
    senStarStopCont++;
    if (senStarStopCont >= 2) {
      senStarStop = sensorBordaDig[0];
    }
  } else {
    senStarStopCont = 0;
  }

//conforme o sensor direito vai detectando as marcas brancas, o "trecho" da pista vai mudando

  if (senCurva == 1 && senCurvaAnt == 0) {
    //Curva detectada;
    Trecho = Trecho + 1;
    Serial.println(Trecho);
    ErSenInt = 0;
    // Acresentar Timer
  }
  if (senStarStop == 1 && senStarStopAnt == 0) {
    //Start Stop detectado;
    StartStop = StartStop + 1;

  }

  if (StartStop == 2) { // número de marcações para parar
    motor.stop_Motor();
    parou = 1;
    digitalWrite(LED_L3, HIGH);
  }

  if (senCurva == 1 || senStarStop == 1) {
    digitalWrite(LED_L1, HIGH);
  } else digitalWrite(LED_L1, LOW);
  //    Buzzer(senCurva || senStarStop); // Buzzer soa enquanto os sensores de borda estiverem detectando a linha

  // Início ---------------------------------------------------------------------------------------------------------------------------------------------------------
  if (Iniciado != 0 && parou == 0) { // Logo depois de apertarbluetooth.println o botão de Start

    digitalWrite(LED_L3, HIGH);

    if (micros() - T_Sen_0 >= 2000) {
      ErSen_0 = ErSen;
      ErSen =  sensorArrayErro;


      if (millis() - T_Parada <= 20000) {
        StartStop = 0;
      }
        
      //bluetooth.println(Trecho);
     // Trecho = 1;        // Teste ---------------------------------------------------
      bluetooth.println("Trecho: ");
      bluetooth.println(Trecho);
      bluetooth.println(trechoTipo[Trecho]);
      
      switch (trechoTipo[0]) {  //Voltar posição do vetor para variável Trecho

        case 'A': // Curva aberta
          KPs = KP_c_aberta;
          KIs = KI_c_aberta;//0.000001 * 256;
          KDs = KD_c_aberta;//400 * 256;
          VELs = Vel_c_aberta*0.06;
          VELerro = Vel_erro_c_aberta;
          //bluetooth.println("A");
          break;

       

        case 'F': // Curva Fechada
        
          KPs = KP_c_fechada;
          KIs = KI_c_fechada;//0.000001 * 256;
          KDs = KD_c_fechada;//400 * 256;
          VELs = Vel_c_fechada*0.08;
          VELerro = Vel_erro_c_fechada;
          //bluetooth.println("F");
          break;

        case 'R': // reta normal
        
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R;
          VELerro = Vel_erro_R;
          //bluetooth.println("F");
          break;

        case 'C': // reta curta
        
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R*0.5;
          VELerro = Vel_erro_R;
          bluetooth.println("C");
          break;

        case 'L': // Reta longa
        default:
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R;
          VELerro = Vel_erro_R;
          //bluetooth.println("L");
          /*if(millis()>1000  ){
            
             KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R*0.5;
          VELerro = Vel_erro_R;
          }*/

           break;
            

      }

//-------------------PID------------------------------------------------------------

  //setamos uma velocidade pra frente e uma velocidade pra trás q vai fazer o robô rodar no proprio eixo (PID)

      U[0] = VELs  - VELerro * (abs (ErSen)/3); // Cálculo da velocidade linear; abs=funçao de módulo e dividimos por 3 
                                                                           //por causa da posição dos sensores (3  a -3)


//qunado o erro for muito grande, ele reduz a velocidade pra não ir muito rápido    
      if (U[0] < 0) U[0] = 0; // Para teste
      
//calculo do PID, tempo q passou(delta T)
      T_Sen_0 = T_Sen_1;
      T_Sen_1 = micros();
      //calculo do delta T
      dT_Sen = (T_Sen_1 - T_Sen_0) / 1000000.0;
      
//parte do integrador, calcula o erro pra parte do integrador, se o erro for maior que 100, ele satura a 100, e limita o erro.

      ErSenInt += ErSen * dT_Sen;

      if (ErSenInt > 100) ErSenInt  = 100;
      else if (ErSenInt < -100) ErSenInt  = -100;
      
//calculo do PID, U[1] é o PID, kp(constante proporcional), KI(integral), KD (derivativa), ai ele depende das variáveis q estão setadas
//tudo q é setado é dividido por delta T (por isso q tem q calcular o delta T), porque precisamos achar a derivada do erro.

      U[1] =  KPs * ErSen + KIs * ErSenInt + KDs * (ErSen - ErSen_0) / dT_Sen ; // Velocidade angular calculada pelo PID

      
//depois calculamos quanto cada roda tem q rodar, porque uma roda precisa rodar no sentido horario e a outra no sentido anti-horario
//U[1] é pra frente, U[0] é pra tras, Uv é a velocidade da roda

         ///  U[0] =0;

      //Uv[0] = U[0] - (dist_L / 4.0) * U[1];
      //Uv[1] = U[0] + (dist_L / 4.0) * U[1];
      Uv[0] = U[0] - U[1] / 2.0;
      Uv[1] = U[0] + U[1] / 2.0;

      if (Uv[0] > 1.0 ) Uv[0] = 1;
      else if (Uv[0] < -1 ) Uv[0] = -1;

      if (Uv[1] > 1 ) Uv[1] = 1;
      else if (Uv[1] < -1 ) Uv[1] = -1;

      //        if (Trecho == 0) {
      //          motorSetVel(Uv[0] * 0.2 * 65535, Uv[1] * 0.2 * 65535);
      //        }else
     motor.motorSetVel(Uv[0] * 100, Uv[1] * 100);// usamos 65535, pq o pwm é de 0 a 2^10-1, ou seja é de 0 a 65355(eu usei o pwm como 125) no caso 
      //porque o Uv foi calcula pra ser um valor entre 0 e 1; quebrado (double)
    
    }
  } else {    // É para as vezes pares que acionar o botão de Start, o robô para e zera a contagem dos sensores de borda

    Trecho = 0;
    motor.stop_Motor();

  }

  

  /*
    Serial3.print(sensorArrayDig[0]);
    Serial3.print(sensorArrayDig[1]);
    Serial3.print(sensorArrayDig[2]);
    Serial3.print(sensorArrayDig[3]);
    Serial3.print(sensorArrayDig[4]);
    Serial3.print(sensorArrayDig[5]);
    Serial3.print(sensorArrayDig[6]);
    Serial3.print(sensorArrayDig[7]);
    Serial3.print(", ");
    Serial3.println(sensorArrayErro);
    Serial3.println(StartStop);
    Serial3.println(trechoTipo[Trecho]);
    Serial3.print(sensorBordaAnalog[0]);
    Serial3.print(", ");
    Serial3.println(sensorBordaAnalog[1]);
  */
}

#define BATMULT 4.25531915  //multiplicador de acordo com os resistores utilizados no meu caso foram 10k e 22k    4.20108696
void verificaBateria() {
  static int lowBat = 0;
  static float bateriaVH[10],
         bateriaVHSum = 0 ;
  bateriaVHSum -= bateriaVH[0];
  for (int ii = 0; ii < 9; ii++) {
    bateriaVH[ii] = bateriaVH[ii + 1];
  }
  bateriaVH[9] =  map(analogRead(analogBat), 0, 4095, 0, 3300) * BATMULT / 1000.0;
  bateriaVHSum += bateriaVH[9];
  bateriaV = bateriaVHSum / 10.0;
  //Serial3.println(bateriaV);
  if (bateriaV < 1.78 && bateriaV > 1.6) { //1.81
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
