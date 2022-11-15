#include "../include/types.h"
#include"../include/testes.h"
#include "../include/motor.h"

Motor mot = Motor();

Testes::Testes() {

}

void Testes::init() {
    mot.motorInit();
    pinMode(BOT1, INPUT);
    pinMode(BOT2, INPUT);
}

void Testes::motor() {

}

void Testes::imu() {
    while (true) {
        Serial.println("Testando IMU");
        mot.motorsControl(80,0);
    }
    
}