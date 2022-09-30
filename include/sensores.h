#include "./types.h"

class Sensores {

  private:

  public:
    Sensores();
    void sensorInit();
    void sensorLer(float *sensorArrayErroptr, int sensorBordaDig[]);
    void sensorCalibrate();
};