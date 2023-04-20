#include "./types.h"
#include <QTRSensors.h>
#include "bluetooth.h"
#include "motor.h"


QTRSensors sensores; //Declaração dos sensores da frente
QTRSensors borda; //Declaração dos sensores de borda

const uint8_t qtd_sensores = 6; //Definição da quantidade de sensores frontais
const uint8_t qtd_borda = 2; //Definição da quantidade de sensores de borda

uint16_t valores_sensor[qtd_sensores]; //Criação do vetor para armazenar os valores lidos pelos sensores frontais
uint16_t valores_borda[qtd_borda]; //Criação do vetor para armazenar os valores lidos pelos sensores de borda


bool calibrado = 0, ligado = 0, parada = 1; //Indica se já foi calibrado e se ta ligado, respectivamente

float Kp = 3, Kd = 0, Ki = 0; //Constantes multiplicativas para o PID

float I = 0, erro_anterior = 0;
int velocidade = 60; //Velocidade para os motores (pode e deve ser ajustada)
uint8_t velocidade_maxima = 70;

int erros[6] = {26, 16, 6, -6, -16, -26}; //Valores dos erros para cada situação de leitura dos sensores
unsigned long int tempo_anterior = 0, tempo_anterior2 = 0, tempo_parada = 0;
uint8_t marcacao_direita = 0, marcacao_esquerda = 0;

Motor motor = Motor();
Bluetooth bluetooth = Bluetooth();


void calibracao();
void leitura();
float PID(float erro);
float calcula_erro();
void marcacoes_laterais();


void setup(){
//Declaração-de-pinos-------------------------------------------------------------
    pinMode(BTN1,INPUT_PULLDOWN);
    pinMode(BTN2,INPUT_PULLDOWN);
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);

    motor.init_motor();
    motor.stop_motor();
    sensores.setTypeAnalog(); //Define os sensores frontais como analógicos
    borda.setTypeRC(); //Define os sensores de borda como digitais 

    borda.setSensorPins((const uint8_t[]){PB10, PA8},qtd_borda); //Definição dos pinos dos sensores de borda (direita esquerda, nessa ordem)
    sensores.setSensorPins((const uint8_t[]){PA6, PA5, PA4, PA3, PA0, PB1}, qtd_sensores); //Definição dos pinos dos sensores frontais, da ESQUERDA PARA DIREITA (lembra que eu observei a Top Layer, ou seja, PA0 = S1)
    Serial.begin(9600); //Inicialização do monitor serial

    if(bluetooth_activate)
        bluetooth.bluetooth_init();
}

void loop(){
    calibracao();
    leitura();
}

void calibracao(){
    while ((!digitalRead(BTN2)) && (calibrado == 0)) { //Fica preso no while até que o botão de calibração seja pressionado
    digitalWrite(LED1, HIGH); //Os leds ficam acesso até que o botão seja pressionado
    digitalWrite(LED2, HIGH); //Os leds ficam acesso até que o botão seja pressionado

    motor.stop_motor();

    if(bluetooth_activate)
        bluetooth.bluetooth_opcoes(); 

    delay(100);

    if(digitalRead(BTN1)){
        Serial.println("calibrando"); //Mensagem exibida no monitor serial indicando calibração
        for (int i = 0; i < 20; i++){ //Aqui tem que ver quantas vezes ele tem q passar pelo processo de calibração
            sensores.calibrate();
    
            digitalWrite(LED1, LOW); //Pisca os leds
            digitalWrite(LED2, LOW); //Pisca os leds
            delay(200);
            digitalWrite(LED1, HIGH); //Pisca os leds
            digitalWrite(LED2, HIGH); //Pisca os leds
            delay(200);
            borda.calibrate();
        }
        calibrado = 1; //Indica que o robô já foi calibrado e pode sair do loop de calibração
        marcacao_direita = 0;
        marcacao_esquerda = 0;
        ligado = 0;
    }
  }
}

void leitura(){
    if(calibrado && (digitalRead(BTN2) or ligado)){
        if(parada){
            tempo_parada = millis();
            parada = 0;
        }
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        sensores.readCalibrated(valores_sensor);
        borda.read(valores_borda);
        ligado = 1;
        marcacoes_laterais();

        if(digitalRead(BTN1)){
            calibrado = 0;
            ligado = 0;
        }
        //Caso seja necessário averiguar os valores lidos pelos sensores, descomente essa parte abaixo:
        
        for (uint8_t i = 0; i < qtd_sensores; i++){
            Serial.print("Sensor");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.println(valores_sensor[i]);
        }
        /*
        for (uint8_t i = 0; i < 2; i++){
            Serial.print("Sensor borda ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(valores_borda[i]);
        }
        delay(250);
        */
        
        motor.speed_motor(PID(calcula_erro()),velocidade,velocidade_maxima);

        if(bluetooth_activate)
            bluetooth.bluetooth_PID(Kp,Kd,Ki,velocidade_maxima,velocidade);
    }
}

float calcula_erro(){ 
    float erro = 0;
    uint8_t cont_sensores = 0; 
    for (uint8_t i = 0; i < 6; i++){
        if(valores_sensor[i] <= 500){
            erro += erros[i];
            cont_sensores++;
        }
    }

    if(cont_sensores == 0 || cont_sensores == 6)
        erro = 0;
    else
        erro = erro/cont_sensores;
    
    //Serial.println(erro);
    erro = constrain(erro,-4000,4000);

    return erro;
}

float PID(float erro){
    float valor_PID, P, D;
    P = erro;
    I += P;
    I = constrain(I,-100,100); 
    D = erro - erro_anterior;
    
    valor_PID = (Kp*P) + (Ki*I) + (Kd*D);
    
    erro_anterior = erro;
/*
    Serial.print("P: ");
    Serial.println(Kp*P);
    Serial.print("I: ");
    Serial.println(Ki*I);
    Serial.print("D: ");
    Serial.println(Kd*D);
*/
    //Serial.print("Valor PID: ");
    //Serial.println(valor_PID);
    return valor_PID;
}

void marcacoes_laterais(){
    if(valores_borda[1] <= 10 && millis()-tempo_anterior2 >= 350){
        tempo_anterior2 = millis();
        marcacao_esquerda++;
    }
    
    if(valores_borda[0] <= 5 && millis()-tempo_anterior >= 350){
        tempo_anterior = millis();
        marcacao_direita++;
    }

    if (millis() - tempo_parada >= 37000){ // número de marcações para parar
        delay(500);
        calibrado = 0;
    }
/*

    if (trechos1[marcacao_esquerda]){
        velocidade = 75;
        velocidade_maxima = 95;
        digitalWrite(LED1,LOW);
        delay(20);
    }else{
        velocidade = 75;
        velocidade_maxima = 95;
        digitalWrite(LED1,HIGH);
        delay(20);
    }
    
*/
}