#include "StepperMotor.h"
#include <Arduino.h>
#include "Settings.h"
#include <HardwareTimer.h>

extern HardwareTimer* stepperTimer;

StepperMotor::StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin)
  : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin) {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  disable();
}

void StepperMotor::enable() {
  digitalWrite(_enPin, LOW);
  _enabled = true;
  if (_speed != 0) stepperTimer->resume();
}

void StepperMotor::disable() {
  digitalWrite(_enPin, HIGH);
  _enabled = false;
  stepperTimer->pause();
}

bool StepperMotor::isEnabled() const {
  return _enabled;
}

void StepperMotor::step() {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(_stepPin, LOW);
}

void StepperMotor::setSpeed(float mmPerMin, float mmPerRev) {
  _speed = constrain(mmPerMin, MIN_SPEED, MAX_SPEED);
  if (_speed == 0) {
    stepperTimer->pause();
    return;
  }

  _stepDelay = (60UL * 1000000UL) / (MOTOR_STEPS_PER_REV * MICROSTEPS * (abs(_speed) / mmPerRev));
  digitalWrite(_dirPin, _speed < 0 ? HIGH : LOW);

  stepperTimer->pause();
  stepperTimer->setMode(1, TIMER_OUTPUT_COMPARE);
  stepperTimer->setPrescaleFactor(72);  // 1 MHz (assuming 72 MHz system clock)
  stepperTimer->setOverflow(_stepDelay, MICROSEC_FORMAT);
  stepperTimer->attachInterrupt([this]() { this->step(); });
  stepperTimer->refresh();
  if (_enabled) stepperTimer->resume();
}

float StepperMotor::getSpeed() const {
  return _speed;
}
