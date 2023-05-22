#include "types.h"
#include <QTRSensors.h>

class Sensores{

    private:
        QTRSensors sensores; //Declaração dos sensores da frente
        QTRSensors borda; //Declaração dos sensores de borda
        bool calibrado = 0;

    public:
        static const int qtd_sensores = 8; //Definição da quantidade de sensores frontais
        static const uint8_t qtd_borda = 2; //Definição da quantidade de sensores de borda
        uint16_t valores_sensor[qtd_sensores]; //Criação do vetor para armazenar os valores lidos pelos sensores frontais
        uint16_t valores_borda[qtd_borda]; //Criação do vetor para armazenar os valores lidos pelos sensores de borda

        Sensores();
        
        void initSensors();
        void setSensorsPins();
        void calibrateSensors();
        bool getCalibrado();
        void readSensors();
        void setCalibrado(bool calibrado);

};