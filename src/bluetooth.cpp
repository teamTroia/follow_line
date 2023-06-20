#include "Bluetooth.h"
#include <SoftwareSerial.h>

SoftwareSerial disp_bluetooth(PB7, PB6);

Bluetooth::Bluetooth(){}

void Bluetooth::bluetooth_init(){
    disp_bluetooth.begin(9600);
}

String Bluetooth::bluetooth_opcoes(){
    if(disp_bluetooth.available()){
        disp_bluetooth.println("Digite uma opcao: ");
        opcao_ajuste = leitura_bluetooth();
    }
    Serial.println(opcao_ajuste);
    return opcao_ajuste;
}

void Bluetooth::bluetooth_PID(float& Kp, float& Kd, float& Ki, uint8_t& velocidade_maxima, int& velocidade){
    if(disp_bluetooth.available()){
        const char* opcao = opcao_ajuste.c_str();
        Serial.println(opcao);
        if(opcao[0] == 'P'){
            disp_bluetooth.println("Digite o valor de Kp: ");
            const char *c = leitura_bluetooth().c_str();
            Kp = atof(c);
            Serial.print("Kp: ");
            Serial.println(Kp,5);
        }
        else if(opcao[0] == 'D'){
            disp_bluetooth.println("Digite o valor de Kd: ");
            const char *c = leitura_bluetooth().c_str();
            Kd = atof(c);
            Serial.print("Kd: ");
            Serial.println(Kd,5);
        }
        else if(opcao[0] == 'I'){
            disp_bluetooth.println("Digite o valor de Ki: ");
            const char *c = leitura_bluetooth().c_str();
            Ki = atof(c);
            Serial.print("Ki: ");
            Serial.println(Ki,5);
        }
        else if(opcao[0] == 'V'){
            disp_bluetooth.println("Digite o valor da velocidade: ");
            const char *c = leitura_bluetooth().c_str();
            velocidade = atoi(c);
            Serial.print("Velocidade: ");
            Serial.println(velocidade,5);
        }
        else if(opcao[0] == 'S'){
            disp_bluetooth.println("Digite o valor da velocidade maxima: ");
            const char *c = leitura_bluetooth().c_str();
            velocidade_maxima = atoi(c);
            Serial.print("Velocidade: ");
            Serial.println(velocidade_maxima,5);
        }
        else{
           disp_bluetooth.println("Opcao nao disponivel!"); 
        }
    }
}

String Bluetooth::leitura_bluetooth(){
    char dado_bluetooth;
    char info[6];
    uint8_t cont = 0;

    while(disp_bluetooth.available()){
        dado_bluetooth = disp_bluetooth.read();
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

void Bluetooth::bluetoothPrintln(double info){
    disp_bluetooth.println(info);
}

void Bluetooth::bluetoothPrintln(String info){
    disp_bluetooth.println(info);
}

void Bluetooth::bluetoothPrint(double info){
    disp_bluetooth.println(info);
}

void Bluetooth::bluetoothPrint(String info){
    disp_bluetooth.println(info);
}