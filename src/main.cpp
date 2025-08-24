#include <Arduino.h>
#include <U8g2lib.h>
#include <GyverEncoder.h>
#include <TMCStepper.h>
#include <HardwareTimer.h>
#include <GyverPID.h>
#include <EEPROM.h>

struct PIDParams {
  uint32_t magic = 0xDEADBEEF;  // уникальный "флаг"
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
};

PIDParams lastTuneResult;
bool showTuneResult = false;
unsigned long tuneResultTimer = 0;

#define THERM_PIN PC5
#define ENCODER_CLK PB10
#define ENCODER_DT PB14
#define ENCODER_BTN PB2
#define HOTEND_HEATER PA1
#define STEP_PIN PB4
#define DIR_PIN PB3
#define EN_PIN PC3

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, PB13, PB15, PB12, PB10);
Encoder enc(ENCODER_CLK, ENCODER_DT, ENCODER_BTN);
bool tuningInProgress = false;

#define SAMPLE_AVG 20
#define TEMP_STEP 5
#define TEMP_MIN 0
#define TEMP_MAX 300
#define DEBOUNCE_DELAY 50
#define DISPLAY_REFRESH 100
#define MOTOR_STEPS_PER_REV 200
#define MICROSTEPS 16
#define DEFAULT_FEEDRATE 0.0f
#define MIN_SPEED -1000.0f
#define MAX_SPEED 1000.0f
#define SPEED_STEP 5.0f

const short temptable_100K[][2] PROGMEM = {
  {    1, 864 }, {   21, 300 }, {   25, 290 }, {   29, 280 },
  {   33, 270 }, {   39, 260 }, {   46, 250 }, {   54, 240 },
  {   64, 230 }, {   75, 220 }, {   90, 210 }, {  107, 200 },
  {  128, 190 }, {  154, 180 }, {  184, 170 }, {  221, 160 },
  {  265, 150 }, {  316, 140 }, {  375, 130 }, {  441, 120 },
  {  513, 110 }, {  588, 100 }, {  734,  80 }, {  856,  60 },
  {  938,  40 }, {  986,  20 }, { 1008,   0 }, { 1018, -20 }
};

HardwareTimer *timer = new HardwareTimer(TIM1);

bool fullStopTriggered = false;
unsigned long holdStartTime = 0;

void savePID(const PIDParams& p) {
  EEPROM.put(0, p);
}

class Thermistor {
  public:
    Thermistor(uint8_t pin) : _pin(pin) {}
    float readAvg() {
      int sum = 0;
      for (int i = 0; i < SAMPLE_AVG; i++) sum += analogRead(_pin);
      _lastRaw = sum / SAMPLE_AVG;
      return computeTemp(_lastRaw);
    }
    int raw() { return _lastRaw; }
  private:
    float computeTemp(int rawADC) {
      for (uint8_t i = 1; i < sizeof(temptable_100K)/sizeof(*temptable_100K); i++) {
        if (pgm_read_word(&temptable_100K[i][0]) > rawADC) {
          float adc1 = pgm_read_word(&temptable_100K[i-1][0]);
          float adc2 = pgm_read_word(&temptable_100K[i][0]);
          float temp1 = pgm_read_word(&temptable_100K[i-1][1]);
          float temp2 = pgm_read_word(&temptable_100K[i][1]);
          return temp1 + (temp2 - temp1) * (rawADC - adc1) / (adc2 - adc1);
        }
      }
      return 0.0;
    }
    uint8_t _pin;
    int _lastRaw = 0;
};

class HotendController {
  public:
    HotendController(uint8_t heaterPin)
      : _heaterPin(heaterPin), _pid(50, 0.3, 200, 10)  // Kp, Ki, Kd, dt (мс)
    {
      pinMode(_heaterPin, OUTPUT);
      analogWrite(_heaterPin, 0);  // STM32 обычно поддерживает analogWrite
      _pid.setDirection(NORMAL);
      _pid.setLimits(0, 255);  // для PWM
    }

    void setTargetTemp(float temp) {
      _targetTemp = constrain(temp, TEMP_MIN, TEMP_MAX);
      _pid.setpoint = _targetTemp;
    }

    void setCurrentTemp(float temp) {
      _currentTemp = temp;
      _pid.input = temp;
    }

    void setPID(float Kp, float Ki, float Kd) {
      _pid.Kp = Kp;
      _pid.Ki = Ki;
      _pid.Kd = Kd;
    }

    void update() {
      float power = _pid.getResult();
      analogWrite(_heaterPin, (int)power);
    }

    float getTargetTemp() const { return _targetTemp; }
    float getCurrentTemp() const { return _currentTemp; }

  private:
    uint8_t _heaterPin;
    float _targetTemp = 0;
    float _currentTemp = 0;
    GyverPID _pid;
};

class PIDAutotuner {
  public:
    PIDAutotuner(uint8_t heaterPin, Thermistor& thermistor)
      : _pin(heaterPin), _therm(thermistor) {
      pinMode(_pin, OUTPUT);
    }

    PIDParams tune(float targetTemp) {
      float maxTemp = -1000;
      float minTemp = 1000;
      unsigned long t1 = 0, t2 = 0;
      bool heating = true;

      digitalWrite(_pin, HIGH);
      unsigned long start = millis();

      while (millis() - start < 20000) {
        float temp = _therm.readAvg();

        if (heating && temp >= targetTemp + 5) {
          digitalWrite(_pin, LOW);
          t1 = millis();
          maxTemp = temp;
          heating = false;
        } else if (!heating && temp <= targetTemp - 5) {
          digitalWrite(_pin, HIGH);
          t2 = millis();
          minTemp = temp;
          break;
        }
      }

      float Pu = (t2 - t1) / 1000.0;
      float Ku = (4.0 * (targetTemp - minTemp)) / (maxTemp - minTemp);

      PIDParams params;
      params.Kp = 0.6 * Ku;
      params.Ki = 1.2 * Ku / Pu;
      params.Kd = 0.075 * Ku * Pu;

      return params;
    }

  private:
    uint8_t _pin;
    Thermistor& _therm;
};

class StepperMotor {
  public:
    StepperMotor(uint8_t stepPin, uint8_t dirPin, uint8_t enPin) 
      : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin) {
      pinMode(_stepPin, OUTPUT);
      pinMode(_dirPin, OUTPUT);
      pinMode(_enPin, OUTPUT);
      disable();
    }
    void enable() {
      digitalWrite(_enPin, LOW);
      _enabled = true;
      timer->resume();
    }
    void disable() {
      digitalWrite(_enPin, HIGH);
      _enabled = false;
      timer->pause();
    }
    bool isEnabled() const { return _enabled; }
    void step() {
      digitalWrite(_stepPin, HIGH);
      delayMicroseconds(2);
      digitalWrite(_stepPin, LOW);
    }
    void setSpeed(float mmPerMin, float mmPerRev) {
      _speed = constrain(mmPerMin, MIN_SPEED, MAX_SPEED);
      _stepDelay = (60 * 1000000) / (MOTOR_STEPS_PER_REV * MICROSTEPS * (abs(_speed) / mmPerRev));
      digitalWrite(_dirPin, _speed < 0 ? HIGH : LOW);
      timer->pause();
      timer->setMode(1, TIMER_OUTPUT_COMPARE);
      timer->setPrescaleFactor(72);
      timer->setOverflow(_stepDelay, MICROSEC_FORMAT);
      timer->attachInterrupt([this]() { this->step(); });
      timer->refresh();
      if (_enabled) timer->resume();
    }
    float getSpeed() const { return _speed; }
  private:
    uint8_t _stepPin, _dirPin, _enPin;
    bool _enabled = false;
    unsigned long _stepDelay = 1000;
    float _speed = DEFAULT_FEEDRATE;
};

Thermistor therm(THERM_PIN);
HotendController hotend(HOTEND_HEATER);
StepperMotor motor(STEP_PIN, DIR_PIN, EN_PIN);

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
  EEPROM.get(0, lastTuneResult);
  if (lastTuneResult.magic != 0xDEADBEEF) {
    // EEPROM пустой — загружаем дефолт
    lastTuneResult = { 0xDEADBEEF, 50.0, 0.3, 200.0 };
    savePID(lastTuneResult);  // сохранить начальные
  }
  hotend.setPID(lastTuneResult.Kp, lastTuneResult.Ki, lastTuneResult.Kd);
}

static unsigned long lastDisplayUpdate = 0;
static unsigned long lastDebounceTime = 0;
static unsigned long blinkTimer = 0;
static bool blinkState = true;
static uint8_t selectedField = 0;
static bool editMode = false;

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  if (tuningInProgress) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor((128 - u8g2.getStrWidth("TUNING...")) / 2, 32);
    u8g2.print("TUNING...");
    u8g2.sendBuffer();
    return;
  }

  u8g2.setCursor((128 - u8g2.getStrWidth("PETnitrator")) / 2, 10);
  u8g2.print("PETnitrator");

  // Температура
  u8g2.setCursor(0, 20);
  u8g2.print("Temp: ");
  u8g2.print(hotend.getCurrentTemp(), 1);
  u8g2.print(" / ");

  String tempStr = String(hotend.getTargetTemp(), 1);
  int tempX = u8g2.getCursorX();
  int tempY = 20;
  int tempW = u8g2.getStrWidth(tempStr.c_str());

  if (selectedField == 0 && (editMode ? blinkState : true)) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(tempX - 1, tempY - 8, tempW + 2, 10);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(tempX, tempY);
  u8g2.print(tempStr);
  u8g2.setDrawColor(1);

  // Скорость
  u8g2.setCursor(0, 30);
  u8g2.print("Srv: ");
  String speedStr = String(motor.getSpeed(), 1);
  int speedX = u8g2.getCursorX();
  int speedY = 30;
  int speedW = u8g2.getStrWidth(speedStr.c_str());

  if (selectedField == 1 && (editMode ? blinkState : true)) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(speedX - 1, speedY - 8, speedW + 2, 10);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(speedX, speedY);
  u8g2.print(speedStr);
  u8g2.setDrawColor(1);

  if (tuningInProgress) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor((128 - u8g2.getStrWidth("TUNING...")) / 2, 32);
    u8g2.print("TUNING...");
  }

  u8g2.print(" mm/min ");
  if (motor.getSpeed() != 0)
    u8g2.print(motor.getSpeed() > 0 ? "->" : "<-");

  u8g2.setCursor(0, 63);
  u8g2.setCursor(0, 40);
  String autotuneLabel = "Autotune PID";
  int atX = u8g2.getCursorX();
  int atY = 40;
  int atW = u8g2.getStrWidth(autotuneLabel.c_str());

  if (selectedField == 2 && (editMode ? blinkState : true)) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(atX - 1, atY - 8, atW + 2, 10);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(atX, atY);
  u8g2.print(autotuneLabel);
  u8g2.setDrawColor(1);
//  u8g2.print("t.me/BaambooClub");
  u8g2.sendBuffer();

  if (showTuneResult) {
    u8g2.setCursor(0, 40);
    u8g2.print("Kp:"); u8g2.print(lastTuneResult.Kp, 1);
    u8g2.setCursor(0, 50);
    u8g2.print("Ki:"); u8g2.print(lastTuneResult.Ki, 1);
    u8g2.setCursor(0, 60);
    u8g2.print("Kd:"); u8g2.print(lastTuneResult.Kd, 1);
  }

}

void loop() {
  static uint8_t clickCount = 0;
  static unsigned long lastClickTime = 0;
  enc.tick();

  if (millis() - blinkTimer >= 500) {
    blinkState = !blinkState;
    blinkTimer = millis();
  }

  hotend.setCurrentTemp(therm.readAvg());
  hotend.update();

  // Кручение энкодера
  if (enc.isTurn()) {
    if (editMode) {
      if (selectedField == 0) {
        float newTemp = hotend.getTargetTemp() + (enc.isRight() ? TEMP_STEP : -TEMP_STEP);
        hotend.setTargetTemp(newTemp);
      } else {
        float newSpeed = motor.getSpeed() + (enc.isRight() ? SPEED_STEP : -SPEED_STEP);
        newSpeed = constrain(newSpeed, MIN_SPEED, MAX_SPEED);
        motor.setSpeed(newSpeed, 8);
        if (newSpeed != 0) motor.enable();
        else motor.disable();
      }
    } else {
      selectedField += enc.isRight() ? 1 : -1;
      if (selectedField > 2) selectedField = 0;
      if (selectedField < 0) selectedField = 2;
    }
  }

  if (clickCount == 3) {
    static PIDAutotuner tuner(HOTEND_HEATER, therm);
    tuningInProgress = true;
    updateDisplay();  // сразу показать "TUNING..."
    lastTuneResult = tuner.tune(hotend.getTargetTemp());
    showTuneResult = true;
    tuneResultTimer = millis();
    tuningInProgress = false;
    hotend.setTargetTemp(0);
    clickCount = 0;
    return;
  }

  // Удержание для входа/выхода из редактирования
  if (enc.isHolded() && (millis() - lastDebounceTime > DEBOUNCE_DELAY)) {
    if (selectedField == 2 && !tuningInProgress) {
      tuningInProgress = true;
      updateDisplay();
  
      static PIDAutotuner tuner(HOTEND_HEATER, therm);
      lastTuneResult = tuner.tune(hotend.getTargetTemp());
      hotend.setPID(lastTuneResult.Kp, lastTuneResult.Ki, lastTuneResult.Kd);
      savePID(lastTuneResult);
      showTuneResult = true;
      tuneResultTimer = millis();
      tuningInProgress = false;
    } else {
      editMode = !editMode;
      if (selectedField == 1) {
        if (editMode) motor.enable();
        else if (motor.getSpeed() == 0) motor.disable();
      }
    }
    lastDebounceTime = millis();
  }

  // Клик: тройной клик — аварийная остановка
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

    if (editMode) {
      editMode = false;
    }

    lastDebounceTime = millis();
  }

  hotend.setCurrentTemp(therm.readAvg());
  hotend.update();

  if (millis() - lastDisplayUpdate >= DISPLAY_REFRESH) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  if (showTuneResult && millis() - tuneResultTimer > 10000) {
    showTuneResult = false;
  }

  delay(10);
}