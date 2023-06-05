#include "types.h"

class Encoder {

    private:
        static int posEsq, posDir;
        int contPosicoes = 0;
        const float raio = 1.58;
        static const int freqEncoder = 15;
        double getDistance(int pos);
        
        
    public:
        Encoder();
        static const int qtdTrechos = 30;
        double posicoesEsq[qtdTrechos];
        double posicoesDir[qtdTrechos];
        static void readEncoderEsq();
        static void readEncoderDir();
        void initEncoder();
        void keepPositon();
        void resetPosition();
};

