#include "./types.h"

class Sensores {

  private:

  public:
    Sensores();
    void sensorInit();
    void sensorLer(float *sensorArrayErro, int sensorBordaDig[]);
    void sensorCalibrate();
};