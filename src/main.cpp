#include "./types.h"
#include <QTRSensors.h>

QTRSensors sensores; //Declaração dos sensores da frente
QTRSensors borda; //Declaração dos sensores de borda

const uint8_t qtd_sensores = 6; //Definição da quantidade de sensores frontais
const uint8_t qtd_borda = 2; //Definição da quantidade de sensores de borda

uint16_t valores_sensor[qtd_sensores]; //Criação do vetor para armazenar os valores lidos pelos sensores frontais
uint16_t valores_borda[qtd_borda]; //Criação do vetor para armazenar os valores lidos pelos sensores de borda

bool calibrado = 0, ligado = 0; //Indica se já foi calibrado e se ta ligado, respectivamente

float Kp = 0, Kd = 0, Ki = 0; //Constantes multiplicativas para o PID
float erro=0, P=0, I=0, D=0, valor_PID=0, erro_anterior=0, I_anterior=0;
int velocidade = 100; //Velocidade para os motores (pode e deve ser ajustada)

int erros[7] = {-3, -2, -1, 0, 1, 2, 3}; //Valores dos erros para cada situação de leitura dos sensores
uint64_t tempo_anterior = 0, tempo_anterior2 = 0;
uint8_t marcacao_direita = 0, marcacao_esquerda = 0;

void calibracao();
void leitura();
void PID();
void calcula_erro();
void motor();
void stop_motor();
void marcacoes_laterais();

void setup(){
//Declaração-de-pinos-------------------------------------------------------------
    pinMode(BTN1,INPUT_PULLDOWN);
    pinMode(BTN2,INPUT_PULLDOWN);
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);
    pinMode(MAIN1,OUTPUT);
    pinMode(MBIN1,OUTPUT);

    stop_motor(); //Para os motores

    sensores.setTypeAnalog(); //Define os sensores frontais como analógicos
    borda.setTypeRC(); //Define os sensores de borda como digitais 

    borda.setSensorPins((const uint8_t[]){PB10, PA8},qtd_borda); //Definição dos pinos dos sensores de borda
    sensores.setSensorPins((const uint8_t[]){PB1, PA6, PA5, PA4, PA3, PA0}, qtd_sensores); //Definição dos pinos dos sensores frontais
    Serial.begin(9600); //Inicialização do monitor serial
}

void loop(){
    calibracao();
    leitura();
    PID();
    motor();
}

void calibracao(){
    while ((!digitalRead(BTN2)) && (calibrado == 0)) { //Fica preso no while até que o botão de calibração seja pressionado
    digitalWrite(LED1, HIGH); //Os leds ficam acesso até que o botão seja pressionado
    digitalWrite(LED2, HIGH); //Os leds ficam acesso até que o botão seja pressionado

    Serial.println("GO"); //A mensagem "Go" é exibida no monitor

    delay(100);

    if(digitalRead(BTN1)){
        Serial.println("calibrando"); //Mensagem exibida no monitor serial indicando calibração
        for (int i = 0; i < 10; i++){ //Aqui tem que ver quantas vezes ele tem q passar pelo processo de calibração
            sensores.calibrate();
    
            digitalWrite(LED1, LOW); //Pisca os leds
            digitalWrite(LED2, LOW); //Pisca os leds
            delay(200);
            digitalWrite(LED1, HIGH); //Pisca os leds
            digitalWrite(LED2, HIGH); //Pisca os leds
            delay(200);
        }
        calibrado = 1; //Indica que o robô já foi calibrado e pode sair do loop de calibração
    }
  }
}

void leitura(){
    if(calibrado && (digitalRead(BTN2) or ligado)){
        sensores.read(valores_sensor);
        borda.read(valores_borda);
        ligado = 1;
        marcacoes_laterais();

        //Caso seja necessário averiguar os valores lidos pelos sensores, descomente essa parte abaixo:
        /*
        for (uint8_t i = 0; i < qtd_sensores; i++){
            Serial.print("Sensor");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_sensor[i]);
        }
        
        for (uint8_t i = 0; i < 2; i++){
            Serial.print("Sensor borda ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_borda[i]);
        }
        delay(250);
        */
       calcula_erro();
    }
}

void calcula_erro(){   
    if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] <= 1000)
        erro = erros[6];
    
    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] <= 1000)
        erro = erros[5];
    
    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] > 1000)
        erro = erros[4];

    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[3];
        
    else if(valores_sensor[0] > 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[2];

    else if(valores_sensor[0] <= 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[1];

    else if(valores_sensor[0] <= 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[0];
}

void PID(){
    P = erro;
    I = I + I_anterior;
    D = erro-erro_anterior;
    
    valor_PID = (Kp*P) + (Ki*I) + (Kd*D);
    
    I_anterior=I;
    erro_anterior=erro;
}

void motor(){
    int vel_esquerdo = velocidade - valor_PID;
    int vel_direito = velocidade + valor_PID;

    vel_esquerdo = constrain(vel_esquerdo,0,255); //Limita o valor da velocidade a no mínimo 0 e no máximo 255
    vel_direito = constrain(vel_direito,0,255); //Limita o valor da velocidade a no mínimo 0 e no máximo 255

    analogWrite(MAIN1,vel_esquerdo);
    analogWrite(MBIN1,vel_direito);
}

void stop_motor(){
    digitalWrite(MAIN1,0);
    digitalWrite(MBIN1,0);
}

void marcacoes_laterais(){
    if(valores_borda[1] <= 300 && millis()-tempo_anterior2 >= 400){
        tempo_anterior2 = millis();
        marcacao_esquerda++;
        //Serial.print("Sensor esquerda: ");
        //Serial.println(marcacao_esquerda);
    }
    
    if(valores_borda[0] <= 400 && millis()-tempo_anterior >= 400){
        tempo_anterior = millis();
        marcacao_direita++;
        //Serial.print("Sensor direita: ");
        //Serial.println(marcacao_direita);
    }
    
}