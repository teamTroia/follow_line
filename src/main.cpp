#include "../include/types.h"
#include "../include/motor.h"
#include "../include/testes.h"


Motor motores = Motor();
Testes testes = Testes();

void setup() {
    Serial.begin(9600);
    motores.motorInit();
    testes.init();
    // testes.motor();
    testes.imu();
}
void loop() {
    
}