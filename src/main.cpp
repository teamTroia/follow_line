#include "./types.h"
#include <QTRSensors.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(PB7, PB6);
QTRSensors sensores; //Declaração dos sensores da frente
QTRSensors borda; //Declaração dos sensores de borda

const uint8_t qtd_sensores = 6; //Definição da quantidade de sensores frontais
const uint8_t qtd_borda = 2; //Definição da quantidade de sensores de borda

uint16_t valores_sensor[qtd_sensores]; //Criação do vetor para armazenar os valores lidos pelos sensores frontais
uint16_t valores_borda[qtd_borda]; //Criação do vetor para armazenar os valores lidos pelos sensores de borda


bool calibrado = 0, ligado = 0; //Indica se já foi calibrado e se ta ligado, respectivamente

float Kp = 1.675, Kd = 9.5, Ki = 0.006; //Constantes multiplicativas para o PID

float erro = 0, P = 0, I = 0, D = 0, valor_PID = 0, erro_anterior = 0;
int velocidade = 80; //Velocidade para os motores (pode e deve ser ajustada)
uint8_t velocidade_maxima = 130;

int erros[6] = {20, 10, 0, 0, -10, -20}; //Valores dos erros para cada situação de leitura dos sensores
uint64_t tempo_anterior = 0, tempo_anterior2 = 0;
uint8_t marcacao_direita = 0, marcacao_esquerda = 0;

String opcao_ajuste = "N";

void calibracao();
void leitura();
void PID();
void calcula_erro();
void motor();
void stop_motor();
void marcacoes_laterais();
void bluetooth_PID();
void bluetooth_opcoes();
String leitura_bluetooth();

void setup(){
//Declaração-de-pinos-------------------------------------------------------------
    pinMode(BTN1,INPUT_PULLDOWN);
    pinMode(BTN2,INPUT_PULLDOWN);
    pinMode(LED1,OUTPUT);
    pinMode(LED2,OUTPUT);
    pinMode(MAIN1,OUTPUT);
    pinMode(MBIN1,OUTPUT);
    pinMode(MAIN2,OUTPUT);
    pinMode(MBIN2,OUTPUT);
    pinMode (PWMA, OUTPUT);
    pinMode (PWMB, OUTPUT);

    stop_motor(); //Para os motores

    sensores.setTypeAnalog(); //Define os sensores frontais como analógicos
    borda.setTypeRC(); //Define os sensores de borda como digitais 

    borda.setSensorPins((const uint8_t[]){PB10, PA8},qtd_borda); //Definição dos pinos dos sensores de borda (direita esquerda, nessa ordem)
    sensores.setSensorPins((const uint8_t[]){PA0, PA3, PA4, PA5, PA6, PB1}, qtd_sensores); //Definição dos pinos dos sensores frontais, da ESQUERDA PARA DIREITA (lembra que eu observei a Top Layer, ou seja, PA0 = S1)
    Serial.begin(9600); //Inicialização do monitor serial
    bluetooth.begin(9600);
}

void loop(){
    calibracao();
    leitura();
}

void calibracao(){
    while ((!digitalRead(BTN2)) && (calibrado == 0)) { //Fica preso no while até que o botão de calibração seja pressionado
    digitalWrite(LED1, HIGH); //Os leds ficam acesso até que o botão seja pressionado
    digitalWrite(LED2, HIGH); //Os leds ficam acesso até que o botão seja pressionado
    stop_motor();
    //Serial.println("GO"); //A mensagem "Go" é exibida no monitor

    bluetooth_opcoes(); 
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

        if(digitalRead(BTN1)){
            calibrado = 0;
            ligado = 0;
        }
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
        bluetooth_PID();
        calcula_erro();
        PID();
        motor();
    }
}

void calcula_erro(){ 
    uint8_t cont_sensores = 0; 
    for (uint8_t i = 0; i < 6; i++){
        if(valores_sensor[i] <= 1000){
            erro += erros[i];
            cont_sensores++;
        }
    }

    if(cont_sensores == 0){
        erro = 0;
    }else{
    erro = erro/cont_sensores;
    erro = constrain(erro,-3000,3000);
    //Serial.println(erro);
    }
}

void PID(){
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

}

void motor(){
    int vel_esquerdo = velocidade - valor_PID;
    int vel_direito = velocidade + valor_PID;

    vel_esquerdo = constrain(vel_esquerdo,-velocidade_maxima,velocidade_maxima); //Limita o valor da velocidade a no mínimo 0 e no máximo 255
    vel_direito = constrain(vel_direito,-velocidade_maxima,velocidade_maxima); //Limita o valor da velocidade a no mínimo 0 e no máximo 255

/*
    Serial.print("Vel_esquerdo: ");
    Serial.println(vel_esquerdo);

    Serial.print("Vel_direito: ");
    Serial.println(vel_direito);
*/
/*
    if(vel_esquerdo > 0){
        digitalWrite(MAIN2,LOW);
        analogWrite(PWMA,vel_esquerdo);
        digitalWrite(MAIN1,HIGH);
    }else if(vel_esquerdo == 0){
        digitalWrite(PWMA,HIGH);
        digitalWrite(MAIN1,HIGH);
        digitalWrite(MAIN2,HIGH);
    }else{
        vel_esquerdo = -vel_esquerdo;
        digitalWrite(MAIN1,LOW);
        digitalWrite(MAIN2,HIGH);
        analogWrite(PWMA,vel_esquerdo);
    }

    if(vel_direito > 0){
        digitalWrite(MBIN2,LOW);
        analogWrite(PWMB,vel_direito);
        digitalWrite(MBIN1,HIGH);
    }else if(vel_direito == 0){
        digitalWrite(PWMB,HIGH);
        digitalWrite(MBIN1,HIGH);
        digitalWrite(MBIN2,HIGH);
    }else{
        vel_direito = -vel_direito;
        digitalWrite(MBIN1,LOW);
        digitalWrite(MBIN2,HIGH);
        analogWrite(PWMB,vel_direito);
    }
*/
}

void stop_motor(){
    digitalWrite(MAIN1,0);
    digitalWrite(MBIN1,0);
    digitalWrite(MAIN2,0);
    digitalWrite(MBIN2,0);
    digitalWrite(PWMA,0);
    digitalWrite(PWMB,0);
}

void marcacoes_laterais(){
    if(valores_borda[1] <= 400 && millis()-tempo_anterior2 >= 400){
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

void bluetooth_PID(){
    if(bluetooth.available()){
        const char* opcao = opcao_ajuste.c_str();
        Serial.println(opcao);
        if(opcao[0] == 'P'){
            bluetooth.println("Digite o valor de Kp: ");
            const char *c = leitura_bluetooth().c_str();
            Kp = atof(c);
            Serial.print("Kp: ");
            Serial.println(Kp,5);
        }
        else if(opcao[0] == 'D'){
            bluetooth.println("Digite o valor de Kd: ");
            const char *c = leitura_bluetooth().c_str();
            Kd = atof(c);
            Serial.print("Kd: ");
            Serial.println(Kd,5);
        }
        else if(opcao[0] == 'I'){
            bluetooth.println("Digite o valor de Ki: ");
            const char *c = leitura_bluetooth().c_str();
            Ki = atof(c);
            Serial.print("Ki: ");
            Serial.println(Ki,5);
        }
        else if(opcao[0] == 'V'){
            bluetooth.println("Digite o valor da velocidade: ");
            const char *c = leitura_bluetooth().c_str();
            velocidade = atoi(c);
            Serial.print("Velocidade: ");
            Serial.println(velocidade,5);
        }
        else{
           bluetooth.println("Opcao nao disponivel!"); 
        }
    }
}

void bluetooth_opcoes(){
    if(bluetooth.available()){
        bluetooth.println("Digite uma opcao: ");
        opcao_ajuste = leitura_bluetooth();
    }
    Serial.println(opcao_ajuste);
}

String leitura_bluetooth(){
    char dado_bluetooth;
    char info[6];
    uint8_t cont = 0;

    while(bluetooth.available()){
        dado_bluetooth = bluetooth.read();
        if(dado_bluetooth != '\n'){
            info[cont] = dado_bluetooth;
            cont++;
        }else{
            info[cont] = '\0';
            cont = 0;
        }
    }
    return info;
}
