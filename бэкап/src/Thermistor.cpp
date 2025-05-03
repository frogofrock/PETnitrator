#include "Thermistor.h"
#include "Settings.h"
#include <Arduino.h>

const short temptable_100K[][2] PROGMEM = {
  {1, 864}, {21, 300}, {25, 290}, {29, 280}, {33, 270}, {39, 260}, {46, 250}, {54, 240},
  {64, 230}, {75, 220}, {90, 210}, {107, 200}, {128, 190}, {154, 180}, {184, 170}, {221, 160},
  {265, 150}, {316, 140}, {375, 130}, {441, 120}, {513, 110}, {588, 100}, {734, 80}, {856, 60},
  {938, 40}, {986, 20}, {1008, 0}, {1018, -20}
};

Thermistor::Thermistor(uint8_t pin) : _pin(pin), _lastRaw(0) {}

float Thermistor::readAvg() {
  int sum = 0;
  for (int i = 0; i < SAMPLE_AVG; i++) {
    sum += analogRead(_pin);
    delay(1);  // гарантирует корректное усреднение
  }
  _lastRaw = sum / SAMPLE_AVG;
  return computeTemp(_lastRaw);
}

float Thermistor::computeTemp(int rawADC) {
  for (uint8_t i = 1; i < sizeof(temptable_100K) / sizeof(*temptable_100K); i++) {
    int adc2 = pgm_read_word(&temptable_100K[i][0]);
    if (adc2 > rawADC) {
      int adc1 = pgm_read_word(&temptable_100K[i - 1][0]);
      int temp1 = pgm_read_word(&temptable_100K[i - 1][1]);
      int temp2 = pgm_read_word(&temptable_100K[i][1]);
      return temp1 + (float)(temp2 - temp1) * (rawADC - adc1) / (adc2 - adc1);
    }
  }
  return pgm_read_word(&temptable_100K[sizeof(temptable_100K) / sizeof(*temptable_100K) - 1][1]);
}
