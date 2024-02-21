#include <types.h>

class EncoderGambi{
    private:
        double velocidade = 35;
        int distancias [30] = {120, 80, 12, 5};
        int aux = 0;
    public:
        EncoderGambi();
        bool verificaDistancia(uint64_t tempo);
        bool alteraVelocidade(uint64_t tempo);
        int retornaDistancia(uint64_t tempo);



};