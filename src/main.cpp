#include "./types.h"
#include "Sensores.h"
#include "Bluetooth.h"
#include "Motor.h"
#include "Encoder.h"
#include "EncoderGambi.h"

float Kp = 31, Kd = 110, Ki = 0.004; //Constantes multiplicativas para o PID

float I = 0, erro_anterior = 0;
int velocidade = 110; //Velocidade para os motores (pode e deve ser ajustada) OBS: 60 da bom
uint8_t velocidade_maxima = 200; //90 deu bom

Motor motor = Motor();
Bluetooth bluetooth = Bluetooth();
Sensores sensor = Sensores();
//EncoderGambi gambi = EncoderGambi();
//Encoder encoder = Encoder();

int erros[sensor.qtd_sensores] = {30, 26, 16, 6, -6, -16, -26, -30}; //Valores dos erros para cada situação de leitura dos sensores
unsigned long int tempo_anterior = 0, tempo_anterior2 = 0;
uint8_t marcacao_direita = 0;


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
    sensor.initSensors();
    //encoder.initEncoder();

    sensor.setSensorsPins();
    Serial.begin(9600); //Inicialização do monitor serial

    if(bluetooth_activate)
        bluetooth.bluetooth_init();
}

void loop(){
    calibracao();
    leitura();
}

void calibracao(){
    while ((!digitalRead(BTN2)) && (!sensor.getCalibrado())) { //Fica preso no while até que o botão de calibração seja pressionado
    digitalWrite(LED1, HIGH); //Os leds ficam acesso até que o botão seja pressionado
    digitalWrite(LED2, HIGH); //Os leds ficam acesso até que o botão seja pressionado

    motor.stop_motor();

    if(bluetooth_activate)
        bluetooth.bluetooth_opcoes(); 

    delay(100);

    if(digitalRead(BTN1)){
        Serial.println("calibrando"); //Mensagem exibida no monitor serial indicando calibração
        sensor.calibrateSensors();
        sensor.setCalibrado(1);
        marcacao_direita = 0;
        motor.setLigado(0);
    }
  }
}

void leitura(){
    if(sensor.getCalibrado() && (digitalRead(BTN2) or motor.getLigado())){
        digitalWrite(LED1,LOW);
        digitalWrite(LED2,LOW);
        sensor.readSensors();
        motor.setLigado(1);
        //encoder.readEncoder();
        marcacoes_laterais();
        // if(gambi.alteraVelocidade(millis()-tempo_anterior2)){
        //     velocidade = 90;
        // }else{
        //     velocidade = 110;
        // }

        if(digitalRead(BTN1)){
            sensor.setCalibrado(0);
            motor.setLigado(0);
        }
    
        motor.speed_motor(PID(calcula_erro()),velocidade,velocidade_maxima);

        if(bluetooth_activate)
            bluetooth.bluetooth_PID(Kp,Kd,Ki,velocidade_maxima,velocidade);
    }
}

float calcula_erro(){ 
    float erro = 0;
    uint8_t cont_sensores = 0; 
    for (uint8_t i = 0; i < sensor.qtd_sensores; i++){
        if(sensor.valores_sensor[i] <= 500){
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
    if(sensor.valores_borda[0] <= 1500 || sensor.valores_borda[1] <= 1500 && millis()-tempo_anterior2 >= 300){
        tempo_anterior2 = millis();
        //encoder.keepPositon();
        //encoder.resetPosition();
        digitalWrite(LED2, HIGH);
        delay(20);
    }
    
    if(sensor.valores_borda[2] <= 1500 && millis()-tempo_anterior >= 300){
        tempo_anterior = millis();
        marcacao_direita++;
        digitalWrite(LED1, HIGH);
        delay(20);
    }

    // if (marcacao_direita >= 5){ // número de marcações para parar
    //     delay(200);
    //     motor.stop_motor();
    //     motor.setLigado(0);
    //     sensor.setCalibrado(0);
        
        // for (int i = 0; i < Encoder::qtdTrechos; i++){
        //     bluetooth.bluetoothPrintln(encoder.posicoes[i]);
        //     delay(50);
        // }
    //}
}