#include "EncoderUI.h"
#include "Settings.h"
#include "HotendController.h"
#include "StepperMotor.h"

extern HotendController hotend;
extern StepperMotor motor;

extern Encoder enc;
extern uint8_t selectedField;
extern bool editMode;

extern unsigned long lastDebounceTime;
extern unsigned long lastClickTime;
extern unsigned long holdStartTime;
extern unsigned long calibrationStart;
extern const unsigned long calibrationTime;

extern bool pidCalibrating;
extern bool heating;
extern bool calibrationDone;
extern float maxTemp;
extern float minTemp;
extern float lastPeakTime;

void handleEncoder() {
  static uint8_t clickCount = 0;

  if (!editMode && enc.isTurn()) {
    selectedField += enc.isRight() ? 1 : -1;
    if (selectedField > 2) selectedField = 0;
    if (selectedField < 0) selectedField = 2;
  }

  if (enc.isTurn() && editMode) {
    if (selectedField == 0) {
      float newTemp = hotend.getTargetTemp() + (enc.isRight() ? TEMP_STEP : -TEMP_STEP);
      hotend.setTargetTemp(newTemp);
    } else if (selectedField == 1) {
      float newSpeed = motor.getSpeed() + (enc.isRight() ? SPEED_STEP : -SPEED_STEP);
      newSpeed = constrain(newSpeed, MIN_SPEED, MAX_SPEED);
      motor.setSpeed(newSpeed, 8);
      if (newSpeed != 0) motor.enable();
      else motor.disable();
    }
  }

  if (enc.isHolded() && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    editMode = !editMode;
    if (selectedField == 1) {
      if (editMode) motor.enable();
      else if (motor.getSpeed() == 0) motor.disable();
    }
    lastDebounceTime = millis();
  }

  if (enc.isClick() && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    if (millis() - lastClickTime < 700) {
      clickCount++;
    } else {
      clickCount = 1;
    }
    lastClickTime = millis();

    if (clickCount >= 3) {
      hotend.setTargetTemp(0);
      motor.setSpeed(0, 8);
      motor.disable();
      clickCount = 0;
      return;
    }

    if (selectedField == 2) {
      pidCalibrating = true;
      heating = true;
      calibrationDone = false;
      maxTemp = 0;
      minTemp = 1000;
      lastPeakTime = millis();
    }

    if (editMode) editMode = false;
    lastDebounceTime = millis();
  }

  if (enc.isHolded() && (millis() - holdStartTime > 5000)) {
    pidCalibrating = true;
    calibrationStart = millis();
    holdStartTime = millis();
  }
}
