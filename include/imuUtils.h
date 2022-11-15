#ifndef IMUUTILS_H
#define IMUUTILS_H

#include "./types.h"

class IMU {
private:

public:
      IMU();
      void init_mpu();
      float readAngularSpeed();
};
#endif