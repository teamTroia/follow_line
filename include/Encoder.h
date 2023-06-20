#include "types.h"

class Encoder {

    private:
        int posEsq, posDir;
        bool estDir = 0, estEsq = 0;
        int contPosicoes = 0;
        const float raio = 1.58;
        const int freqEncoder = 43;
        double getDistance(int pos);
        void readEncoderDir();
        void readEncoderEsq();
        
        
    public:
        Encoder();
        static const int qtdTrechos = 36;
        double posicoes[qtdTrechos];
        void initEncoder();
        void keepPositon();
        void resetPosition();
        void readEncoder();
};

