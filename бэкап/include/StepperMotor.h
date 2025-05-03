#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>
#include <HardwareTimer.h>

#define MOTOR_STEPS_PER_REV 200
#define MICROSTEPS 16
#define MIN_SPEED -1000.0f
#define MAX_SPEED 1000.0f
#define DEFAULT_FEEDRATE 0.0f

extern HardwareTimer* stepperTimer;

class StepperMotor {
  public:
    StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin);
    void enable();
    void disable();
    bool isEnabled() const;
    void step();
    void setSpeed(float mmPerMin, float mmPerRev);
    float getSpeed() const;
  private:
    uint8_t _stepPin, _dirPin, _enPin;
    bool _enabled = false;
    unsigned long _stepDelay = 1000;
    float _speed = DEFAULT_FEEDRATE;
};

#endif
