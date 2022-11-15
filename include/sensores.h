#ifndef SENSORES_H
#define SENSORES_H

#include "./types.h"
#include <SoftwareSerial.h>

class Sensores {

  private:

  public:
    Sensores();
    void sensorInit();
    void sensorLer(float &sensorArrayErro, int sensorBordaDig[], int &sSoma);
    void sensorCalibrate();
};

#endif