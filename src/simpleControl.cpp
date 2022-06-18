#include "../include/simpleControl.h"
#include "../include/types.h"
#include "../include/motor.h"

bool pretoVerify (treshold treshold, uint16 valorLido) {
    if ((valorLido > pivo) && (valorLido < treshold.max)) {
        return false;
    }else if ((valorLido < pivo) && (valorLido > treshold.min)) {
        return true;
    }
    return false;
}
SimpleControl::SimpleControl() {

}

void SimpleControl::main(treshold tresholds[]) {
    Motor motores = Motor();
    uint16 valoresLidos[8];
    uint8 pinosSensores[8] = {S1,S2,S3,S4,S5,S6,S7,S8};

    for (int i = 0; i < 8; i++) {
        valoresLidos[i] =  analogRead(pinosSensores[i]);
        Serial.println(pretoVerify(tresholds[i], valoresLidos[i]));
    }
    if (!pretoVerify(tresholds[2],valoresLidos[2]) && !pretoVerify(tresholds[3],valoresLidos[3]) && !pretoVerify(tresholds[4],valoresLidos[4]) && !pretoVerify(tresholds[5],valoresLidos[5])) {
        if (!pretoVerify(tresholds[6],valoresLidos[6])) {
            motores.setB(-20);
        }else if (!pretoVerify(tresholds[1],valoresLidos[1])) {
            motores.setA(-20);
        }else {
            motores.setAll(40,40);
        }
    }else {
        motores.setAll(40,40);
    }
    return;
}