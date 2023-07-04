#include <types.h>

class EncoderGambi{
    private:
        double velocidade = 16.42;
        int distancias [30] = {5, 20, 12, 5};
        int aux = 0;
    public:
        EncoderGambi();
        bool verificaDistancia(uint64_t tempo);
        bool alteraVelocidade(uint64_t tempo);



};