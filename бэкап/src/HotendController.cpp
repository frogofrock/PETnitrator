#include "HotendController.h"
#include <Arduino.h>
#include "Settings.h"

HotendController::HotendController(uint8_t heaterPin) : _heaterPin(heaterPin) {
  pinMode(_heaterPin, OUTPUT);
  digitalWrite(_heaterPin, LOW);
}

void HotendController::setTargetTemp(float temp) {
  _targetTemp = constrain(temp, TEMP_MIN, TEMP_MAX);
}

void HotendController::setCurrentTemp(float temp) {
  _currentTemp = temp;
}

float HotendController::getTargetTemp() const {
  return _targetTemp;
}

float HotendController::getCurrentTemp() const {
  return _currentTemp;
}
