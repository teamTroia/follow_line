#ifndef sensorLinha_H
#define sensorLinha_H

#include "./types.h"

class SensorLinha {
private:
    treshold tresholds;
    
public:
    SensorLinha(treshold tresholds);
    bool itsBlack (uint16 valor);

};

#endif