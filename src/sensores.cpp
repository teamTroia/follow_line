#include "./types.h"
#include "../include/sensores.h"

//Variáveis----------------------------------------------------------------

//const int sensorArrayPin[8] = {PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0};//matriz com os nossos 8 sensores de linha
const int sensorBordaPin[2] = {PB10, PA8};

const float sensorArrayErroConst[8] = { 3, 1.75, 1.75, 0.5, -0.5, 1.75, -1.75, -3};//posição dos sensores varia de 3 a -3

int sensorArrayAnalog[8]; //Valor Lido do Array de sensores
int sensorBordaAnalog[2]; //Valor Lido dos sensores de borda

int sensorBordaThreshold[2] = {300, 100}; //Valor de limiar dos sensores de borda

int sensorArrayDig[8]; //Valor Lido do Array de sensores


int contFalhasConsecutivas = 0;

bool sCruzamento = 0;
unsigned long int sCruzamentoTempo = 0;


//Calibração

//0bs: maior valor lido= preto; menor valor lido=branco; abaixo de 3100 é branco
int maiorArrayAnalog[8] = {4200, 4200, 4200, 4200, 4200, 4200, 4200, 4200}; // Maior valor medido pelo array de sensores (valores já pré-determinados para evitar erros na falta da calibração)
int menorArrayAnalog[8] = {900, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // Menor valor medido pelo array de sensores
int superiorThreshold[8] = {4095, 3900, 4095, 4095, 4095, 4095, 4095, 4095};//Maiores valores reais lidos e tirados através de testes
int inferiorThreshold[8] = {2020, 700, 1000, 1000, 1000, 1100, 1200, 2600};//Menores valores reais lidos e tirados através de testes

int maiorBordaAnalog[2] = {4000, 4000};
int menorBordaAnalog[2] = {1500, 1500};
int superiorThresholdBorda[2] = {300, 100};
int inferiorThresholdBorda[2] = {300, 100};

int faixaLeituraArray[8];
int faixaLeituraBorda[2];
int faixaAtuacaoArray[8];
int faixaAtuacaoBorda[2];
int Calibrado = 0;


unsigned short int sensorCalib[8][256];

Sensores::Sensores () {

}
//Funções------------------------------------------------------------------
void Sensores::sensorInit() {
  pinMode(MULTIPLEX_COM,INPUT_ANALOG );
  pinMode(MULTIPLEX_SWITCH_A, OUTPUT);
  pinMode(MULTIPLEX_SWITCH_B, OUTPUT);
  pinMode(MULTIPLEX_SWITCH_C,OUTPUT);

  if (BORDA_RC) {
    pinMode(sensorBordaPin[0], INPUT);
    pinMode(sensorBordaPin[1], INPUT);
  } else {
    pinMode(sensorBordaPin[0], INPUT_ANALOG);
    pinMode(sensorBordaPin[1], INPUT_ANALOG);
  }
  Serial.println("Sensores inicializados!");
}

int sensorRead (int sensorNum) {
  
  int valor1,valor2,valor3;
  valor1 = 001 & sensorNum;
  valor2 = ((010 & sensorNum) >> 1);
  valor3 = ((100 & sensorNum) >> 2);

  digitalWrite(MULTIPLEX_SWITCH_A, valor1 ? HIGH : LOW);
  digitalWrite(MULTIPLEX_SWITCH_B, valor2 ? HIGH : LOW);
  digitalWrite(MULTIPLEX_SWITCH_C, valor3 ? HIGH : LOW);
  return analogRead(MULTIPLEX_COM);
}

// Calibração
void Sensores::sensorCalibrate() {

  digitalWrite(LED_L3, HIGH);                       // indicação do início da calibração
  delay(1000);

  if (Calibrado == 0) {
    for (int i = 0; i < 8; i++) {                   // "zerando" valores dos sensores para adquirir novos sem limitações, apenas na primeira calibração,
      maiorArrayAnalog[i] = 0;                      // nas seguintes o valor será apenas atualizado
      menorArrayAnalog[i] = 10000;
    }
    for (int i = 0; i < 2; i++) {                   // "zerando" valores dos sensores para adquirir novos sem limitações, apenas na primeira calibração,
      maiorBordaAnalog[i] = 0;                      // nas seguintes o valor será apenas atualizado
      menorBordaAnalog[i] = 10000;
    }
    Calibrado = 1;
  }
  //digitalWrite(LED_L3, LOW);                      //indicação do início da calibração

  for (int j = 0; j < 4000; j++) {
    for (int i = 0; i < 8; i++) {
      sensorArrayAnalog[i] = sensorRead(i);
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(sensorArrayAnalog[i]);

      if (sensorArrayAnalog[i] > maiorArrayAnalog[i]) {
        maiorArrayAnalog[i] = sensorArrayAnalog[i];
      }
      if (sensorArrayAnalog[i] < menorArrayAnalog[i]) {
        menorArrayAnalog[i] = sensorArrayAnalog[i];
      }
    }
    for (int k = 0; k < 2; k++) {
      sensorBordaAnalog[k] = analogRead(sensorBordaPin[k]);
      if (sensorBordaAnalog[k] > maiorBordaAnalog[k]) {
        maiorBordaAnalog[k] = sensorBordaAnalog[k];
      }
      if (sensorBordaAnalog[k] < menorBordaAnalog[k]) {
        menorBordaAnalog[k] = sensorBordaAnalog[k];
      }
    }

    delay(1);
    if (j % 80 == 0) {
      digitalWrite(LED_L3, !digitalRead (LED_L3));                      // LED piscando durante a calibração
    }
  }

  for (int L = 0; L < 8; L++) {
    faixaLeituraArray[L] = maiorArrayAnalog[L] - menorArrayAnalog[L];            // definição da faixa de valores colhidos
    faixaAtuacaoArray[L] = (faixaLeituraArray[L]) / divisor;                     // divisão da faixa para definição de limites
    superiorThreshold[L] = maiorArrayAnalog[L] - (faixaAtuacaoArray[L]);         // definição de limite superior (preto)
    inferiorThreshold[L] = menorArrayAnalog[L] + (4 * faixaAtuacaoArray[L]);     // definição de limite inferior (branco)

  }
  for (int H = 0; H < 2; H++) {
    faixaLeituraBorda[H] = maiorBordaAnalog[H] - menorBordaAnalog[H];
    faixaAtuacaoBorda[H] = (faixaLeituraBorda[H]) / divisor;
    superiorThresholdBorda[H] = maiorArrayAnalog[H] - (faixaAtuacaoBorda[H]);
    inferiorThresholdBorda[H] = menorArrayAnalog[H] + (4 * faixaAtuacaoBorda[H]);
  }
  digitalWrite(LED_L3, HIGH);
  delay(1000);
  digitalWrite(LED_L3, LOW);                      // indicação do fim da calibração
  
  Serial.print("superiorThreshold[] = {");
  for (int L = 0; L < 8; L++) {
    Serial.print(superiorThreshold[L]);
    Serial.print(", ");
  }
  Serial.println("};");
  Serial.print("inferiorThreshold[] = {");
  for (int L = 0; L < 8; L++) {
    Serial.print(inferiorThreshold[L]);
    Serial.print(", ");
  }
  Serial.println("};");
  delay(5000);
}

int sCont;//contador 
float sSoma;//soma da posição (sensorArrayErroConst[8])
int sii;//variavel usada para fazer a ontagem dos sensores   
unsigned long auxTemp;

//----------------------- Leitura dos sensores----------------------
void Sensores::sensorLer(float *sensorArrayErroptr, int sensorBordaDig[]) {
  sSoma = 0;
  sCont = 0;
  //Array
  for (sii = 0; sii < 8; sii++) {
    sensorArrayAnalog[sii] = sensorRead(sii);

    if (INVERTER) {//como foi declarado INVERTER, vamos declarar como verdadeiro se está no preto
      if (sensorArrayDig[sii] == 0 && sensorArrayAnalog[sii] > superiorThreshold[sii]) {
        sensorArrayDig[sii] = 1;//VERDADEIRO, é preto
        Serial.println("branco");
      }
      if (sensorArrayDig[sii] == 1 && sensorArrayAnalog[sii] < inferiorThreshold[sii]) {
        sensorArrayDig[sii] = 0;//FALSO; não é preto
        Serial.println("preto");
      }
    }
    else {
      if ( sensorArrayAnalog[sii] > superiorThreshold[sii]) {
        sensorArrayDig[sii] = 0;
        Serial.print("Sensor ");
        Serial.print(sii);
        Serial.print(" = ");
        Serial.println("PRETO");
      }
      if ( sensorArrayAnalog[sii] < inferiorThreshold[sii]) {
        sensorArrayDig[sii] = 1;//se estiver no branco, é verdadeiro
        Serial.print("Sensor ");
        Serial.print(sii);
        Serial.print(" = ");
        Serial.println("BRANCO");
      }
    }
    //delay(500);

    //essa parte soma a posição dos sensores e descobre o valor de erro e é usado no PID
    if (sensorArrayDig[sii]) {
      sSoma += sensorArrayErroConst[sii];
      sCont++;
    }
  }
  // Calcular erro e verificar se perder a faixa no array
  
  if (sCont > 0 && sCont <= 4) {
    *sensorArrayErroptr = sSoma / float(sCont);
    contFalhasConsecutivas = 0;
  } else if (sCont > 4) {
    *sensorArrayErroptr = *sensorArrayErroptr;
    contFalhasConsecutivas = 0;
  } else {
    contFalhasConsecutivas++;
    if (contFalhasConsecutivas > 10) {
      if (*sensorArrayErroptr < 0) *sensorArrayErroptr = -4;// o follow está posicionado mais para o lado (t???)
      if (*sensorArrayErroptr > 0) *sensorArrayErroptr =  4;// o follow está posicionado mais para o lado (t???)
    }
  }

 /*--------------------------------------------------------------------------------*/
  // Cruzamento
  #warning ("CRUZAMENTO DETECTADO PELO NUMERO DE SENSORES DO ARRAY QUE DETECTAM BRANCO, VERIFICAR SE AINDA FUNCIONA NA PLACA NOVA")
  if (sCont > 5) { 
    sCruzamento = 1;
    sCruzamentoTempo = millis();
  }
  if (sCruzamento && (millis() - sCruzamentoTempo) > 270) {
    sCruzamento = 0;
  }

  // Borda
  if (sCruzamento == 1) {
    sensorBordaAnalog[0] = 0;
    sensorBordaAnalog[1] = 0;
    sensorBordaDig[0] = 0;
    sensorBordaDig[1] = 0;

  } else if (BORDA_RC) { /*leitura DIGITAL dos sensores de BORDA*/
    pinMode(sensorBordaPin[0], OUTPUT);
    pinMode(sensorBordaPin[1], OUTPUT);
    digitalWrite(sensorBordaPin[0], HIGH);
    digitalWrite(sensorBordaPin[1], HIGH);
    auxTemp = micros();
    while ((micros() - auxTemp) < 10); // Esperar por 10us
       
    sensorBordaAnalog[0] = 0;
    sensorBordaAnalog[1] = 0;
    auxTemp = micros();
    pinMode(sensorBordaPin[0], INPUT);
    pinMode(sensorBordaPin[1], INPUT);
    while ((sensorBordaAnalog[0] == 0 || sensorBordaAnalog[1] == 0) && (micros() - auxTemp) < BORDA_RC_TIMEOUT) {
      if (!digitalRead(sensorBordaPin[0]) && sensorBordaAnalog[0] == 0 ) {
        sensorBordaAnalog[0] = micros() - auxTemp;
      }
      if (!digitalRead(sensorBordaPin[1]) && sensorBordaAnalog[1] == 0 ) {
        sensorBordaAnalog[1] = micros() - auxTemp;
      }
    }

    if (sensorBordaAnalog[0] == 0) sensorBordaAnalog[0] = 65535;//TA FALANDO QUE SE A LEITURA FOR FALSA, TA NO PRETO; 65535 é o PWM em binário
    if (sensorBordaAnalog[1] == 0) sensorBordaAnalog[1] = 65535;

    sensorBordaDig[0] = (sensorBordaAnalog[0] < sensorBordaThreshold[0]);
    sensorBordaDig[1] = (sensorBordaAnalog[1] < sensorBordaThreshold[1]);
  } else {
    sensorBordaAnalog[0] = analogRead(sensorBordaPin[0]);
    sensorBordaAnalog[1] = analogRead(sensorBordaPin[1]);
    sensorBordaDig[0] = (sensorBordaAnalog[0] < sensorBordaThreshold[0]);
    sensorBordaDig[1] = (sensorBordaAnalog[1] < sensorBordaThreshold[1]);
  }
  // Analogico para para digital das bordas
  if (INVERTER) {
    sensorBordaDig[0] = !sensorBordaDig[0];
    sensorBordaDig[1] = !sensorBordaDig[1];
  }
}


/*
   HISTÓRICO DE DADOS COLHIDOS
   Maiores
  {4005, 3997, 3976, 3992, 3995, 3996, 3991, 4013}
  {3947, 3940, 3907, 3952, 3961, 3884, 3955, 3979}
  {4002, 3994, 3991, 3991, 3992, 3991, 3992, 3999}

   Menores
  {1663, 1753, 1797, 1753, 1903, 1761, 1679, 2081}
  {1307, 1451, 1651, 1611, 1757, 1641, 1433, 1921}
  {1427, 1495, 1833, 2070, 2102, 1813, 1551, 1753}


*/