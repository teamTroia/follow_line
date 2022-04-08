 /*  Pandemia-2020-2021
    Follow line utilizando STM32F103C8T6 - TROIA
    por: Luara Linhares e Fidelis ihuuuu
*/

// PID --------------------------------------------------------------
char trechoTipo[] = { 'R', 'F', 'R', 'F', 'R'}; //TREINO
//char trechoTipo[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9'}; //OFICIAL

float KP_R = 0.8, // CORRIGE MAIS RÁPIDO MAS CAUSA INSTABILIDADE -------------- 0.6 ---- 0.7
      KI_R = 0.0, // CORRIGE NO LONGO TEMPO ---------------------------------- 0.01 --- 0.005
      KD_R = 0.66, // CORRIGE MAIS RÁPIDO ------------------------------------ 0.015 -- 0.00
      Vel_R = 0.65, // -------------------------------------------------------- 0.05 --- 0.05
      Vel_erro_R = 0.5; // ----------------esse de curva----------------------------------- 0.08 --- 0.15

float KP_c_aberta = 0.01,
      KI_c_aberta = 0.0,
      KD_c_aberta = 0.00,
      Vel_c_aberta = 0.00425,
      Vel_erro_c_aberta = 0.002;

float KP_c_fechada = 0.33,
      KI_c_fechada = 0.15,
      KD_c_fechada = 0.80,
      Vel_c_fechada = 0.2,
      Vel_erro_c_fechada = 0.5;

//#Define------------------------------------------------------------
#define LED_L1 PA9
#define LED_L2 PB4
#define LED_L3 PB15
#define BOT1 PA15
#define BOT2 PB3
#define dist_L 0.150 // distância entre rodas
#define analogBat PB1   // Pino do leitor de bateria

//#Include-----------------------------------------------------------
#include "motor.h"
#include "sensores.h"



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

unsigned long int T_Sen_0, T_Sen_1, T_Bat, T_Parada;
float KPs, KIs, KDs, VELs, VELerro, dT_Sen;

float bateriaV;  //Tensão da Bateria
float LOWBAT_LEVEL = 1.78;
boolean LOWBAT = false, // verdadeiro para indicar a bateria fraca
        parou = 0;

void verificaBateria();

void setup() {
  motorInit();
  pinMode(BOT1, INPUT_PULLUP);
  pinMode(BOT2, INPUT_PULLUP);
  pinMode(LED_L1, OUTPUT);
  pinMode(LED_L3, OUTPUT);
  pinMode(analogBat, INPUT_ANALOG);
  //pinMode(MOSFET, OUTPUT);
  //pinMode(BUZZER_PIN, OUTPUT);

  Serial3.begin(9600);
  sensorInit();

  ErSen = ErSenInt = 0;

  T_Sen_0 = T_Sen_1 = micros();

  Trecho = 0;
  StartStop = 0;
  U[0] = 0; // velocidade linear
  U[1] = 0; // velocidade angular

}

void loop() {

  //bot2 parou de funfar, depois tem q apagar essA GAMBI

  if (digitalRead(BOT1)) {
    B++;
    }

  if (digitalRead(BOT2)) {
    
    digitalWrite(LED_L1, HIGH); //1
    delay(50);
    //digitalWrite(LED_L1, LOW);
    delay(50);
    digitalWrite(LED_L3, HIGH);
    Serial.println("GO");
    delay(50);
    //digitalWrite(LED_L3, LOW);

    
    parou = 0;

    Iniciado += 1;
    T_Sen_0 = T_Sen_1 = micros();

    delay(100);
    digitalWrite(LED_L1, HIGH); //1
    delay(50);
    digitalWrite(LED_L1, LOW);
    delay(50);
    digitalWrite(LED_L3, HIGH);
    delay(50);
    digitalWrite(LED_L3, LOW);
    delay(500);
    digitalWrite(LED_L1, HIGH); //2
    delay(50);
    digitalWrite(LED_L1, LOW);
    delay(50);
    digitalWrite(LED_L3, HIGH);
    delay(50);
    digitalWrite(LED_L3, LOW);
    delay(500);
    digitalWrite(LED_L1, HIGH);
    digitalWrite(LED_L3, HIGH);
    delay(200);
    digitalWrite(LED_L1, LOW);
    digitalWrite(LED_L3, LOW);

    T_Parada = millis();
  } else if (digitalRead(BOT1) && Iniciado == 0 ) {       //condição calibração
    sensorCalibrate();
  }
 
  sensorLer();
  
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
    if (senStarStopCont >= 3) {
      senStarStop = sensorBordaDig[0];
    }
  } else {
    senStarStopCont = 0;
  }

//conforme o sensor direito vai detectando as marcas brancas, o "trecho" da pista vai mudando

  if (senCurva == 1 && senCurvaAnt == 0) {
    //Curva detectada;
    Trecho = Trecho + 1;
    ErSenInt = 0;
    // Acresentar Timer
  }
  if (senStarStop == 1 && senStarStopAnt == 0) {
    //Start Stop detectado;
    StartStop = StartStop + 1;

  }

  if (StartStop == 2) { // número de marcações para parar
    stop_Motor();
    parou = 1;
  }

  if (Trecho == 2) { // número de marcações para parar
    stop_Motor();
    parou = 1;
  }
 
  if (senCurva == 1 || senStarStop == 1) {
    digitalWrite(LED_L1, HIGH);
  } else digitalWrite(LED_L1, LOW);
  //    Buzzer(senCurva || senStarStop); // Buzzer soa enquanto os sensores de borda estiverem detectando a linha

  // Início ---------------------------------------------------------------------------------------------------------------------------------------------------------
  if (Iniciado != 0 && parou == 0) { // Logo depois de apertar o botão de Start

    digitalWrite(LED_L3, HIGH);

    if (micros() - T_Sen_0 >= 2000) {
      ErSen_0 = ErSen;
      ErSen =  sensorArrayErro;


      if (millis() - T_Parada <= 20000) {
        StartStop = 0;
      }
        

     // Trecho = 1;        // Teste ---------------------------------------------------

      switch (trechoTipo[1]) {  //Voltar posição do vetor para variável Trecho

        case 'A': // Curva aberta
          KPs = KP_c_aberta;
          KIs = KI_c_aberta;//0.000001 * 256;
          KDs = KD_c_aberta;//400 * 256;
          VELs = Vel_c_aberta;
          VELerro = Vel_erro_c_aberta;
          break;

       

        case 'F': // Curva Fechada
          KPs = KP_c_fechada;
          KIs = KI_c_fechada;//0.000001 * 256;
          KDs = KD_c_fechada;//400 * 256;
          VELs = Vel_c_fechada;
          VELerro = Vel_erro_c_fechada;
          break;

        case 'R': // Reta
        default:
          KPs = KP_R;
          KIs = KI_R;//0.000001 * 256;
          KDs = KD_R;//400 * 256;
          VELs = Vel_R;
          VELerro = Vel_erro_R;
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
      motorSetVel(Uv[0] * 4900, Uv[1] * 4900);// usamos 65535, pq o pwm é de 0 a 2^10-1, ou seja é de 0 a 65355(eu usei o pwm como 125) no caso 
      //porque o Uv foi calcula pra ser um valor entre 0 e 1; quebrado (double)

    }
  } else {    // É para as vezes pares que acionar o botão de Start, o robô para e zera a contagem dos sensores de borda

    Trecho = 0;
    stop_Motor();

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
