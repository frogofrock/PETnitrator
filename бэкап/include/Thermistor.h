#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <Arduino.h>

#define SAMPLE_AVG 20

extern const short temptable_100K[][2];

class Thermistor {
  public:
    Thermistor(uint8_t pin);
    float readAvg();
    int raw();  // Вернуть последнее значение raw
  private:
    float computeTemp(int rawADC);
    uint8_t _pin;
    int _lastRaw = 0;
};

#endif
