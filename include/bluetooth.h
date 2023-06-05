#include "types.h"

class Bluetooth {
    
    private:
        String opcao_ajuste = "N";
        String leitura_bluetooth();
        
    public:
        Bluetooth();
        void bluetooth_PID(float& Kp, float& Kd, float& Ki, uint8_t& velocidade_maxima, int& velocidade);
        String bluetooth_opcoes();
        void bluetooth_init();
        void bluetoothPrintln(double info);
        void bluetoothPrintln(String info);
};