#ifndef HOTEND_CONTROLLER_H
#define HOTEND_CONTROLLER_H
#pragma once

#include <Arduino.h>
#include "Settings.h"

class HotendController {
  public:
    HotendController(uint8_t heaterPin);
    void setTargetTemp(float temp);
    void setCurrentTemp(float temp);
    void update();
    float getTargetTemp() const;
    float getCurrentTemp() const;
  private:
    uint8_t _heaterPin;
    float _targetTemp = 0;
    float _currentTemp = 0;
};

#endif
