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

float Kp = 0.012, Kd = 0, Ki = 0; //Constantes multiplicativas para o PID


float erro = 0, P = 0, I = 0, D = 0, valor_PID = 0, erro_anterior = 0;
int velocidade = 40; //Velocidade para os motores (pode e deve ser ajustada)

int erros[11] = {5000, 4000, 3000, 2000, 1000, 0, -1000, -2000, -3000, -4000, -5000}; //Valores dos erros para cada situação de leitura dos sensores
uint64_t tempo_anterior = 0, tempo_anterior2 = 0;
uint8_t marcacao_direita = 0, marcacao_esquerda = 0;

void calibracao();
void leitura();
void PID();
void calcula_erro();
void motor();
void stop_motor();
void marcacoes_laterais();
void bluetooth_PID();
String leitura_bluetooth();

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

    bluetooth_PID();
    //Serial.println("GO"); //A mensagem "Go" é exibida no monitor

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
        calcula_erro();
        PID();
        motor();
    }
}

void calcula_erro(){   
    if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] <= 1000)
        erro = erros[10];
    
    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] <= 1000)
        erro = erros[9];

    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] <= 1000)
        erro = erros[8];
    
    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] > 1000)
        erro = erros[7];

    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] > 1000)
        erro = erros[6];
    
    else if(valores_sensor[0] > 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] <= 1000 &&valores_sensor[5] > 1000)
        erro = erros[5];

    else if(valores_sensor[0] > 1000 && valores_sensor[1] > 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[5];
        
    else if(valores_sensor[0] > 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] <= 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[4];

    else if(valores_sensor[0] > 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[3];

    else if(valores_sensor[0] <= 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] <= 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[2];

    else if(valores_sensor[0] <= 1000 && valores_sensor[1] <= 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[1];

    else if(valores_sensor[0] <= 1000 && valores_sensor[1] > 1000 && valores_sensor[2] > 1000 && valores_sensor[3] > 1000 && valores_sensor[4] > 1000 &&valores_sensor[5] > 1000)
        erro = erros[0];
    
    else{
        //Serial.println("saiu");
    }

    //Serial.println(erro);
}

void PID(){
    P = erro;
    I += P;
    D = erro-erro_anterior;
    
    valor_PID = (Kp*P) + (Ki*I) + (Kd*D);
    
    erro_anterior = erro;
/*
    Serial.println("P: ");
    Serial.print(Kp*P);
    Serial.println("I: ");
    Serial.print(Ki*I);
    Serial.println("D: ");
    Serial.print(Kd*D);
*/
    //Serial.print("Valor PID: ");
    //Serial.println(valor_PID);

}

void motor(){
    int vel_esquerdo = velocidade + valor_PID;
    int vel_direito = velocidade - valor_PID;

    vel_esquerdo = constrain(vel_esquerdo,0,40); //Limita o valor da velocidade a no mínimo 0 e no máximo 255
    vel_direito = constrain(vel_direito,0,40); //Limita o valor da velocidade a no mínimo 0 e no máximo 255

    //Serial.print("Vel_esquerdo: ");
    //Serial.println(vel_esquerdo);

    //Serial.print("Vel_direito: ");
    //Serial.println(vel_direito);

    analogWrite(MAIN1,vel_esquerdo);
    analogWrite(MBIN1,vel_direito);
}

void stop_motor(){
    digitalWrite(MAIN1,0);
    digitalWrite(MBIN1,0);
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
        char opcao = 'P';
        Serial.println(opcao);
        if(opcao == 'P'){
            bluetooth.println("Digite o valor de Kp: ");
            const char *c = leitura_bluetooth().c_str();
            Kp = atof(c);
            Serial.print("Kp ");
            Serial.println(Kp,5);
        }
        else if(opcao == 'D'){
            bluetooth.println("Digite o valor de Kd: ");
            const char *c = leitura_bluetooth().c_str();
            Kd = atof(c);
            Serial.print("Kd ");
            Serial.println(Kd,5);
        }
        else if(opcao == 'I'){
            bluetooth.println("Digite o valor de Ki: ");
            const char *c = leitura_bluetooth().c_str();
            Ki = atof(c);
            Serial.print("Ki ");
            Serial.println(Ki,5);
        }
        else{
           bluetooth.println("Opcao nao disponivel!"); 
        }
    }
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
