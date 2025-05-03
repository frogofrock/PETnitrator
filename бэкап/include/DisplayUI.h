#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <U8g2lib.h>
#include "HotendController.h"
#include "StepperMotor.h"

extern U8G2_ST7920_128X64_F_SW_SPI u8g2;
extern HotendController hotend;
extern StepperMotor motor;
extern bool editMode;
extern bool blinkState;
extern uint8_t selectedField;

void updateDisplay(bool calibrating, bool calibrationDone);

#endif
