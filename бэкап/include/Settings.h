#ifndef SETTINGS_H
#define SETTINGS_H

// --- Пины энкодера ---
#define ENCODER_CLK PB10
#define ENCODER_DT  PB14
#define ENCODER_BTN PB2

// --- Термодатчик и нагрев ---
#define THERM_PIN     PC5
#define HOTEND_HEATER PA1

// --- Шаговик ---
#define STEP_PIN PB4
#define DIR_PIN  PB3
#define EN_PIN   PC3

// --- EEPROM ---
#define PID_KP_ADDR    0
#define PID_KI_ADDR    4
#define PID_KD_ADDR    8
#define PID_TEMP_ADDR 12

// --- Температурные ограничения ---
#define TEMP_MIN 0
#define TEMP_MAX 300

// --- Настройки шаговика ---
#define MOTOR_STEPS_PER_REV 200
#define MICROSTEPS          16
#define MIN_SPEED          -1000.0f
#define MAX_SPEED           1000.0f
#define DEFAULT_FEEDRATE       0.0f

// --- Прочее ---
#define SAMPLE_AVG        20
#define TEMP_STEP          5
#define SPEED_STEP         5.0f
#define DEBOUNCE_DELAY    50
#define DISPLAY_REFRESH  100

#endif
