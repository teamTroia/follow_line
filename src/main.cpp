
/*
          ||||||||||||||||||||||||||||||||||||||||||||||||||||||||
          ||                                                    ||
          ||      |---------------------------------------|     ||
          ||      |   Código Follow Line (Agora vai)      |     ||
          ||      |---------------------------------------|     ||
          ||                                                    ||
          ||||||||||||||||||||||||||||||||||||||||||||||||||||||||
          || Baseado no código de:                              ||
          ||    Fidelis Melo Júnior                             ||
          ||    Camila Couto                                    ||
          || Refatorado por:                                    ||
          ||    Luara Linhares                                  ||
          ||    João Vitor Santos Barbosa (Sid)                 ||
          ||||||||||||||||||||||||||||||||||||||||||||||||||||||||
          ||                                                    ||
          ||  Repositório original disponivél no github         ||
          ||  Acessível pelo link:                              ||
          ||  https://github.com/teamTroia/follow_line          ||
          ||                                                    ||
          ||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/



#include "../include/types.h"
#include "../include/sensores.h"
#include "../include/motor.h"
#include "../include/sensorBorda.h"
#include "../include/funcsAux.h"
#include "../include/simpleControl.h"

treshold tresholds[8];
uint8 bordaCont = 0;
uint32 auxTemp = micros();

//------------------------------//
//    Iniciação de objetos      //
//------------------------------//

Sensores sensores = Sensores();
Motor motores = Motor();
SensorBorda senBord = SensorBorda();
FuncsAux funcsAux = FuncsAux();

//------------------------------//
//    Iniciação de objetos      //
//------------------------------//

void setup() {
  Serial.begin(9600);
  if (DEBUGMODE) {
    Serial.println("Iniciando...");
  }
  
  motores.init();
  sensores.init();

  for (int i = 0; i < 8; i++) { //Seta tresholds padrão pro caso de não calibrar
    tresholds[i].min = pivo-50;
    tresholds[i].max = pivo+50;
  }
  
  while(digitalRead(BOT2) == LOW) {
    if(digitalRead(BOT1)) {
      sensores.sensorCalibrate(tresholds);
    }
  }
  if (DEBUGMODE) {
    Serial.println("Sensores Calibrados!");
  }
  funcsAux.estadoLeds(false);
  delay(500);
  funcsAux.estadoLeds(true);
  delay(500);
  funcsAux.estadoLeds(false);
  delay(500);
  funcsAux.estadoLeds(true);
  delay(500);
}
void loop() {
  FuncsAux funcsAux = FuncsAux();
  
  if (DEBUGMODE) {
    Serial.println("Entrou no loop");
  }

  
  bool valorBorda[2];
    
  senBord.ler(valorBorda);
  if(DEBUGMODE) {
        Serial.print("Valor borda esquerda: ");
        Serial.println(valorBorda[0]);
        Serial.print("Valor borda direita: ");
        Serial.println(valorBorda[1]);
    }
  if (valorBorda[0] == 1) {
    if (auxTemp - micros() >= 10000000) {
      auxTemp = 0;
      //bordaCont++;
    }
  }
  if (bordaCont >= 2) {
     funcsAux.fim(motores);
  }else {
     if (SIMPLECONTROL) {
      SimpleControl simpleControl = SimpleControl();
      simpleControl.main(tresholds);
     }
   }
}