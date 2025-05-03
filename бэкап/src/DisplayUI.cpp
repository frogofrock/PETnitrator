#include "DisplayUI.h"
#include "HotendController.h"
#include "StepperMotor.h"
#include <U8g2lib.h>
#include <GyverEncoder.h>

extern U8G2_ST7920_128X64_F_SW_SPI u8g2;
extern HotendController hotend;
extern StepperMotor motor;
extern uint8_t selectedField;
extern bool editMode;
extern bool blinkState;

void updateDisplay(bool calibrating, bool calibrationDone) {
  u8g2.clearBuffer();
  u8g2.setCursor((128 - u8g2.getStrWidth("PETnitrator")) / 2, 10);
  u8g2.print("PETnitrator");

  // Температура
  u8g2.setCursor(0, 20);
  u8g2.print("Temp: ");
  u8g2.print(hotend.getCurrentTemp(), 1);
  u8g2.print(" / ");
  if (selectedField == 0 && (editMode ? blinkState : true)) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(u8g2.getCursorX(), 12, 30, 10);
    u8g2.setDrawColor(0);
  }
  u8g2.print(hotend.getTargetTemp(), 1);
  u8g2.setDrawColor(1);

  // Скорость
  u8g2.setCursor(0, 30);
  u8g2.print("Speed: ");
  if (selectedField == 1 && (editMode ? blinkState : true)) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(u8g2.getCursorX(), 22, 30, 10);
    u8g2.setDrawColor(0);
  }
  u8g2.print(motor.getSpeed(), 1);
  u8g2.setDrawColor(1);

  // Меню
  if (selectedField == 2) {
    u8g2.drawStr(0, 40, "> PID Autotune");
  } else {
    u8g2.drawStr(0, 40, "  PID Autotune");
  }

  if (calibrating) {
    u8g2.setCursor(0, 55);
    u8g2.print("Calibrating...");
  }

  if (calibrationDone) {
    u8g2.setCursor(0, 55);
    u8g2.print("Calibration done!");
  }

  u8g2.setCursor(0, 63);
  u8g2.print("t.me/BaambooClub");
  u8g2.sendBuffer();
}
