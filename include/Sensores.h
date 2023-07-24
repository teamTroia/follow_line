#include "types.h"

class Sensores{

    private:
        bool calibrado = 0;

    public:
        static const int qtd_sensores = 8; //Definição da quantidade de sensores frontais
        static const uint8_t qtd_borda = 3; //Definição da quantidade de sensores de borda

        uint8_t sensorPins[qtd_sensores] = {PA7, PA6, PA5, PA4, PA3, PA0, PB1, PA2};
        uint8_t bordaPins[qtd_borda] = {PB10, PB12, PA8};
        
        uint16_t * minimum = nullptr;
        uint16_t * maximum = nullptr;

        uint16_t valores_sensor[qtd_sensores]; //Criação do vetor para armazenar os valores lidos pelos sensores frontais
        uint16_t valores_borda[qtd_borda]; //Criação do vetor para armazenar os valores lidos pelos sensores de borda

        Sensores();
        
        void initSensors();
        void calibrateSensors();
        bool getCalibrado();
        void readSensors();
        void readCalibrated();
        void readBorda();
        void setCalibrado(bool calibrado);

};