#include "../include/types.h"
#include "../include/sensorLinha.h"

SensorLinha::SensorLinha(treshold tresholds) {
  tresholds.max = tresholds.max;
  tresholds.min = tresholds.min;
}
/*
  Verifica se o valor lido é preto ou não
*/
bool SensorLinha::itsBlack (uint16 valor) {

  if ((valor <= tresholds.max)&& (valor >= pivo)) {
    return true;
  }else if ((valor >= tresholds.min)&& (valor <= pivo)) {
    return false;
  }
  return true;
}