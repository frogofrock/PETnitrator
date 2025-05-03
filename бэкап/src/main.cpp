#include <Arduino.h>
#include <U8g2lib.h>
#include <GyverEncoder.h>
#include <GyverPID.h>
#include <EEPROM.h>

#include "Settings.h"
#include "Thermistor.h"
#include "HotendController.h"
#include "StepperMotor.h"
#include "DisplayUI.h"
#include "EncoderUI.h"

#define MENU_PID_AUTOTUNE 2

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, PB13, PB15, PB12, PB10);
Encoder enc(ENCODER_CLK, ENCODER_DT, ENCODER_BTN);

Thermistor therm(THERM_PIN);
HotendController hotend(HOTEND_HEATER);
StepperMotor motor(STEP_PIN, DIR_PIN, EN_PIN);
GyverPID pid(1, 1, 1, 100);
HardwareTimer* stepperTimer = new HardwareTimer(TIM2);

bool fullStopTriggered = false;
unsigned long holdStartTime = 0;
unsigned long lastDisplayUpdate = 0;
bool blinkState = true;
unsigned long blinkTimer = 0;
uint8_t selectedField = 0;
bool editMode = false;
bool pidCalibrating = false;
unsigned long calibrationStart = 0;
const unsigned long calibrationTime = 30000;
unsigned long lastDebounceTime = 0;
unsigned long lastClickTime = 0;
bool calibrationDone = false;
bool heating = false;

float maxTemp = 0;
float minTemp = 1000;
float Tu = 0;
int peakCount = 0;
float totalTu = 0;
float lastPeakTemp = 0;
float lastPeakTime = 0;
unsigned long lastImpulseTime = 0;

unsigned long pwmLastTime = 0;
const unsigned long pwmPeriod = 40;
int pwmOutput = 0;

enum AutoTuneState { IDLE, IMPULSE, PEAK_WAIT };
AutoTuneState autoTuneState = IDLE;

void savePIDandTemp(float kp, float ki, float kd, float temp) {
  EEPROM.put(PID_KP_ADDR, kp);
  EEPROM.put(PID_KI_ADDR, ki);
  EEPROM.put(PID_KD_ADDR, kd);
  EEPROM.put(PID_TEMP_ADDR, temp);
}

void loadPIDandTemp() {
  float kp, ki, kd, temp;
  EEPROM.get(PID_KP_ADDR, kp);
  EEPROM.get(PID_KI_ADDR, ki);
  EEPROM.get(PID_KD_ADDR, kd);
  EEPROM.get(PID_TEMP_ADDR, temp);

  if (!isnan(kp) && !isnan(ki) && !isnan(kd) && !isnan(temp)) {
    pid.Kp = kp;
    pid.Ki = ki;
    pid.Kd = kd;
    pid.setpoint = hotend.getTargetTemp();
    hotend.setTargetTemp(temp);
  }
}

void performAutoTune() {
  float current = hotend.getCurrentTemp();
  float target = hotend.getTargetTemp();
  unsigned long now = millis();

  switch (autoTuneState) {
    case IDLE:
      digitalWrite(HOTEND_HEATER, LOW);
      break;

    case IMPULSE:
      digitalWrite(HOTEND_HEATER, HIGH);
      if (now - lastImpulseTime >= 1500) {
        digitalWrite(HOTEND_HEATER, LOW);
        autoTuneState = PEAK_WAIT;
        lastPeakTemp = current;
      }
      break;

    case PEAK_WAIT:
      if (current < lastPeakTemp) {
        float thisTu = (now - lastImpulseTime) / 1000.0;
        totalTu += thisTu;
        lastImpulseTime = now;
        peakCount++;

        if (peakCount >= 5) {
          Tu = totalTu / peakCount;
          float Ku = (4.0 * 255.0) / (PI * (maxTemp - minTemp));
          float kp = 0.2 * Ku;
          float ki = 2 * kp / Tu;
          float kd = kp * Tu / 3;

          pid.Kp = kp;
          pid.Ki = ki;
          pid.Kd = kd;

          savePIDandTemp(kp, ki, kd, target);
          pidCalibrating = false;
          calibrationDone = true;
          peakCount = 0;
          totalTu = 0;
          autoTuneState = IDLE;
        } else {
          autoTuneState = IMPULSE;
        }
      } else {
        if (current > maxTemp) maxTemp = current;
        if (current < minTemp) minTemp = current;
        lastPeakTemp = current;
      }
      break;
  }
}

void encoderISR() {
  enc.tick();
}

void setup() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);

  enc.setType(TYPE2);
  enc.setTickMode(MANUAL);
  enc.setFastTimeout(15);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setContrast(100);

  motor.setSpeed(DEFAULT_FEEDRATE, 8);
  motor.enable();

  loadPIDandTemp();
  pid.setDirection(NORMAL);
  pid.setLimits(0, 255);

  pinMode(HOTEND_HEATER, OUTPUT);
}

void loop() {
  enc.tick();
  handleEncoder();

  if (millis() - blinkTimer >= 500) {
    blinkState = !blinkState;
    blinkTimer = millis();
  }

  float rawTemp = therm.readAvg();
  float filteredTemp = hotend.getCurrentTemp() * 0.9 + rawTemp * 0.1;
  hotend.setCurrentTemp(filteredTemp);

  if (pidCalibrating) {
    performAutoTune();
  } else {
    pid.input = hotend.getCurrentTemp();
    pid.setpoint = hotend.getTargetTemp();
    pid.getResult();

    unsigned long now = millis();
    unsigned long onTime = (unsigned long)((pid.output / 255.0) * pwmPeriod);
    if ((now - pwmLastTime) < onTime) {
      digitalWrite(HOTEND_HEATER, HIGH);
    } else {
      digitalWrite(HOTEND_HEATER, LOW);
    }
    if (now - pwmLastTime >= pwmPeriod) {
      pwmLastTime = now;
    }
  }

  if (millis() - lastDisplayUpdate >= DISPLAY_REFRESH) {
    updateDisplay(pidCalibrating, calibrationDone);
    lastDisplayUpdate = millis();
  }

  delay(10);
}
