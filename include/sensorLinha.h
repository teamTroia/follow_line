#include "./types.h"

class SensorLinha {
private:
    treshold tresholds;
    
public:
    SensorLinha(treshold tresholds);
    bool itsBlack (uint16 valor);

};