#ifndef sensor_H
#define sensor_H

#include "./types.h"

class Sensores {
private:

public:
    Sensores();
    void init();
    void sensorCalibrate (treshold tresholds[]);
};

#endif