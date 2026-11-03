/*
  Motor Health Monitoring System - ESP32  (FINAL)
  Sensors : INA219 (V, I, P), DS18B20 (temperature), MPU6050 (vibration), Hall sensor (RPM)
  Output  : Serial Monitor (115200 baud) + 3 LEDs
  LEDs    : GREEN = health good, YELLOW = normal, RED = warning

  PINS (as wired)
    I2C SDA = GPIO21, I2C SCL = GPIO22   (INA219 0x40, MPU6050 0x68)
    DS18B20 DATA = GPIO4 (4.7k pull-up to 3V3)
    Hall sensor OUT = GPIO18
    Green LED = GPIO5, Yellow LED = GPIO19, Red LED = GPIO23

  This version does NOT stop if a sensor is missing. It tells you which sensor
  was not found and keeps working with the sensors that are present.
  Send  c  in the Serial Monitor to calibrate again.
*/

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------- Pins ----------
#define PIN_SDA      21
#define PIN_SCL      22
#define PIN_DS18B20   4
#define PIN_HALL     18
#define LED_GREEN     5
#define LED_YELLOW   19
#define LED_RED      23

// ---------- Settings ----------
#define PULSES_PER_REV   1
#define CAL_SAMPLES      10
#define MOTOR_ON_MA      30.0
#define REPORT_MS        1000

#define TEMP_GOOD_MAX    45.0
#define TEMP_NORMAL_MAX  55.0

#define CUR_GOOD_X       1.3
#define CUR_NORMAL_X     1.6
#define RPM_GOOD_DROP    0.10
#define RPM_NORMAL_DROP  0.25
#define VIB_GOOD_X       1.5
#define VIB_NORMAL_X     2.5
#define VIB_GOOD_OFF     0.02
#define VIB_NORMAL_OFF   0.05
#define SUPPLY_LOW_V     6.5

// ---------- Objects ----------
Adafruit_INA219 *ina = nullptr;
Adafruit_MPU6050 mpu;
OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensor(&oneWire);

bool inaOK = false, mpuOK = false, tempOK = false;

// ---------- Hall sensor ----------
volatile uint32_t pulseCount = 0;
volatile uint32_t firstPulseUs = 0;
volatile uint32_t lastPulseUs = 0;
volatile uint32_t totalPulses = 0;

void IRAM_ATTR hallISR() {
  uint32_t now = micros();
  if (now - lastPulseUs > 3000) {
    if (pulseCount == 0) firstPulseUs = now;
    lastPulseUs = now;
    pulseCount++;
    totalPulses++;
  }
}

// RPM = (pulses / (time in s x pulses per rev)) x 60
float readRPM() {
  noInterrupts();
  uint32_t c = pulseCount;
  uint32_t f = firstPulseUs;
  uint32_t l = lastPulseUs;
  pulseCount = 0;
  interrupts();
  if (c < 2) return 0;
  float periodUs = (float)(l - f) / (float)(c - 1);
  return 60000000.0f / (periodUs * PULSES_PER_REV);
}

// ---------- Vibration (RMS of acceleration, gravity removed), in g ----------
float readVibration() {
  const int N = 100;
  float mag[N];
  float sum = 0;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    mag[i] = sqrt(a.acceleration.x * a.acceleration.x +
                  a.acceleration.y * a.acceleration.y +
                  a.acceleration.z * a.acceleration.z) / 9.80665f;
    sum += mag[i];
    delay(3);
  }
  float mean = sum / N;
  float sq = 0;
  for (int i = 0; i < N; i++) {
    float d = mag[i] - mean;
    sq += d * d;
  }
  return sqrt(sq / N);
}

// ---------- LEDs ----------
void setLEDs(int level) {            // -1 off, 0 green, 1 yellow, 2 red
  digitalWrite(LED_GREEN,  level == 0);
  digitalWrite(LED_YELLOW, level == 1);
  digitalWrite(LED_RED,    level == 2);
}

void ledTest() {
  setLEDs(0); delay(400);
  setLEDs(1); delay(400);
  setLEDs(2); delay(400);
  setLEDs(-1);
}

// ---------- Calibration ----------
bool calibrating = true;
int calCount = 0, nI = 0, nR = 0, nV = 0;
float sumI = 0, sumRPM = 0, sumVib = 0;
float baseI = 0, baseRPM = 0, baseVib = 0;

void startCalibration() {
  calibrating = true;
  calCount = 0; nI = nR = nV = 0;
  sumI = sumRPM = sumVib = 0;
  baseI = baseRPM = baseVib = 0;
  setLEDs(-1);
  Serial.println();
  Serial.println(">>> CALIBRATION: run the motor with NO extra load and keep it steady <<<");
}

// ---------- Levels (0 good, 1 normal, 2 warning) ----------
int levelCurrent(float i) {
  if (i <= baseI * CUR_GOOD_X + 10) return 0;
  if (i <= baseI * CUR_NORMAL_X + 20) return 1;
  return 2;
}
int levelRPM(float rpm) {
  float drop = (baseRPM - rpm) / baseRPM;
  if (drop <= RPM_GOOD_DROP) return 0;
  if (drop <= RPM_NORMAL_DROP) return 1;
  return 2;
}
int levelVib(float v) {
  if (v <= baseVib * VIB_GOOD_X + VIB_GOOD_OFF) return 0;
  if (v <= baseVib * VIB_NORMAL_X + VIB_NORMAL_OFF) return 1;
  return 2;
}
int levelTemp(float t) {
  if (t < TEMP_GOOD_MAX) return 0;
  if (t < TEMP_NORMAL_MAX) return 1;
  return 2;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=====================================");
  Serial.println(" Motor Health Monitoring System");
  Serial.println("=====================================");

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  setLEDs(-1);
  Serial.println("LED test: green, yellow, red ...");
  ledTest();

  // ----- I2C scan -----
  Wire.begin(PIN_SDA, PIN_SCL);
  Serial.println("I2C scan:");
  int found = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x");
      Serial.println(a, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  NO I2C device found. Check 3V3, GND, SDA(GPIO21), SCL(GPIO22) and soldered pins.");
  }

  // ----- INA219 (tries all possible addresses) -----
  byte addrs[] = {0x40, 0x41, 0x44, 0x45};
  for (int i = 0; i < 4 && !inaOK; i++) {
    Adafruit_INA219 *t = new Adafruit_INA219(addrs[i]);
    if (t->begin()) {
      ina = t;
      inaOK = true;
      ina->setCalibration_32V_2A();
      Serial.print("INA219  : OK at 0x");
      Serial.println(addrs[i], HEX);
    } else {
      delete t;
    }
  }
  if (!inaOK) Serial.println("INA219  : NOT FOUND (current/voltage will show 0)");

  // ----- MPU6050 (0x68, then 0x69) -----
  if (mpu.begin(0x68) || mpu.begin(0x69)) {
    mpuOK = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    Serial.println("MPU6050 : OK");
  } else {
    Serial.println("MPU6050 : NOT FOUND (vibration will show 0)");
  }
  Wire.setClock(400000);

  // ----- DS18B20 -----
  pinMode(PIN_DS18B20, INPUT);
  Serial.print("GPIO4 level before search: ");
  Serial.println(digitalRead(PIN_DS18B20) ? "HIGH (pull-up OK)" : "LOW (3V3/pull-up not reaching, or DATA shorted to GND)");
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  if (tempSensor.getDeviceCount() > 0) {
    tempOK = true;
    tempSensor.requestTemperatures();
    Serial.println("DS18B20 : OK");
  } else {
    Serial.println("DS18B20 : NOT FOUND (temperature will show N/A)");
  }

  // ----- Hall -----
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);
  Serial.println("Hall    : ready on GPIO18 (turn the shaft / move magnet to test)");

  delay(800);
  startCalibration();
}

// ---------- Loop ----------
void loop() {
  unsigned long start = millis();

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') startCalibration();
  }

  // ----- Read sensors -----
  float busV = 0, current = 0, powerMW = 0;
  if (inaOK) {
    busV = ina->getBusVoltage_V() + ina->getShuntVoltage_mV() / 1000.0f;
    current = ina->getCurrent_mA();
    if (current < 0) current = -current;
    powerMW = ina->getPower_mW();
    if (powerMW < 0) powerMW = -powerMW;
  }

  float temp = -127;
  bool tempValid = false;
  if (tempOK) {
    temp = tempSensor.getTempCByIndex(0);
    tempSensor.requestTemperatures();
    tempValid = (temp > -100);
  }

  float rpm = readRPM();
  float vib = mpuOK ? readVibration() : 0;

  // ----- Print -----
  Serial.println("-------------------------------------");
  Serial.print("Supply voltage : "); if (inaOK) { Serial.print(busV, 2); Serial.println(" V"); } else Serial.println("N/A");
  Serial.print("Current        : "); if (inaOK) { Serial.print(current, 1); Serial.println(" mA"); } else Serial.println("N/A");
  Serial.print("Power          : "); if (inaOK) { Serial.print(powerMW, 1); Serial.println(" mW"); } else Serial.println("N/A");
  Serial.print("Temperature    : "); if (tempValid) { Serial.print(temp, 1); Serial.println(" C"); } else Serial.println("N/A");
  Serial.print("RPM            : "); Serial.println(rpm, 0);
  Serial.print("Vibration (RMS): "); if (mpuOK) { Serial.print(vib, 3); Serial.println(" g"); } else Serial.println("N/A");
  Serial.print("Hall pulses    : "); Serial.println(totalPulses);

  // Motor running: current if INA219 present, otherwise RPM
  bool motorOn = (inaOK && current > MOTOR_ON_MA) || (rpm > 0);

  // ----- Calibration -----
  if (calibrating) {
    if (motorOn) {
      if (inaOK && current > MOTOR_ON_MA) { sumI += current; nI++; }
      if (rpm > 0) { sumRPM += rpm; nR++; }
      if (mpuOK) { sumVib += vib; nV++; }
      calCount++;
      Serial.print("CALIBRATING    : "); Serial.print(calCount);
      Serial.print(" / "); Serial.println(CAL_SAMPLES);
      if (calCount >= CAL_SAMPLES) {
        baseI   = nI ? sumI / nI : 0;
        baseRPM = nR ? sumRPM / nR : 0;
        baseVib = nV ? sumVib / nV : 0;
        calibrating = false;
        Serial.println(">>> CALIBRATION DONE. Normal values: <<<");
        Serial.print("    Normal current : "); Serial.print(baseI, 1);   Serial.println(" mA");
        Serial.print("    Normal RPM     : "); Serial.println(baseRPM, 0);
        Serial.print("    Normal vib     : "); Serial.print(baseVib, 3); Serial.println(" g");
      }
    } else {
      Serial.println("Waiting for the motor to run (current or RPM must be detected)...");
    }
    while (millis() - start < REPORT_MS) delay(5);
    return;
  }

  // ----- Motor off -----
  if (!motorOn) {
    Serial.println("STATUS         : MOTOR OFF");
    setLEDs(-1);
    while (millis() - start < REPORT_MS) delay(5);
    return;
  }

  // ----- Status -----
  int worst = 0;
  String reason = "";

  int lc = (inaOK && baseI > 0)   ? levelCurrent(current) : 0;
  int lr = (baseRPM > 0)          ? levelRPM(rpm) : 0;
  int lv = (mpuOK && baseVib > 0) ? levelVib(vib) : 0;
  int lt = tempValid              ? levelTemp(temp) : 0;

  if (lc > worst) worst = lc;
  if (lr > worst) worst = lr;
  if (lv > worst) worst = lv;
  if (lt > worst) worst = lt;
  if (inaOK && busV < SUPPLY_LOW_V && worst < 1) worst = 1;

  if (lc == 2) reason += "[High current] ";
  if (lr == 2) reason += "[Low RPM] ";
  if (lv == 2) reason += "[High vibration] ";
  if (lt == 2) reason += "[High temperature] ";
  if (inaOK && busV < SUPPLY_LOW_V) reason += "[Supply low] ";

  setLEDs(worst);

  Serial.print("STATUS         : ");
  if (worst == 0)      Serial.println("HEALTH GOOD  (GREEN LED)");
  else if (worst == 1) Serial.println("NORMAL  (YELLOW LED)");
  else                 Serial.println("WARNING  (RED LED)");
  if (reason.length() > 0) {
    Serial.print("Reason         : ");
    Serial.println(reason);
  }

  while (millis() - start < REPORT_MS) delay(5);
}
