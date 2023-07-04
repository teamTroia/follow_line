#include "types.h"

class Giroscopio{
    private:
        const int MPU = 0x68;
        float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;
    public:
        void init();
        void read();
};