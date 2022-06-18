#ifndef sensorBorda_H
#define sensorBorda_H

#include "./types.h"

class SensorBorda {
private:
    treshold tresholds;
public:
    SensorBorda();
    void ler(bool valorBorda[]);
};

#endif