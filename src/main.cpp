
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

treshold tresholds[8];
uint8 bordaCont = 0;

void setup() {
  Serial.begin(9600);
  if (DEBUGMODE) {
    Serial.println("Iniciando...");
  }
  Sensores sensores = Sensores();
  Motor motores = Motor();

  motores.init();
  
  sensores.init();

  for (int i = 0; i < 8; i++) { //Seta tresholds padrão pro caso de não calibrar
    tresholds[i].min = 2000;
    tresholds[i].max = 4000;
  }
  
  while(digitalRead(BOT2) == LOW) {
    if(digitalRead(BOT1)) {
      sensores.sensorCalibrate(tresholds);
    }
  }
  if (DEBUGMODE) {
    Serial.println("Sensores Calibrados!");
  }
}
void loop() {
  if (DEBUGMODE) {
    Serial.println("Entrou no loop");
  }
  Sensores sensores = Sensores();
  SensorBorda senBord = SensorBorda();
  //Motor motores = Motor();

  
  bool valorBorda[2];
    
  senBord.ler(valorBorda);
  if(DEBUGMODE) {
        Serial.print("Valor borda esquerda: ");
        Serial.println(valorBorda[0]);
        Serial.print("Valor borda direita: ");
        Serial.println(valorBorda[1]);
    }
  if (valorBorda[0] == 1) {
    bordaCont++;
  }
  if (bordaCont >= 2) {
    if (DEBUGMODE) {
      Serial.print("ACAAAABBBBOOOOUUUUU!!!!!");
    }
    //motores.stopAll();
  }else {
    //Resto do codigo do follow aqui
  }
  
  

}