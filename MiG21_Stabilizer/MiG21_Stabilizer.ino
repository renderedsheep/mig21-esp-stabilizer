/*
  MiG-21 ESP32 Flight Stabilizer  v1.1   (WORK IN PROGRESS, UNTESTED)
  ------------------------------------------------------------------
  Copyright (C) 2026 Fernand G.
  SPDX-License-Identifier: GPL-3.0-or-later
  This program is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version. It is distributed WITHOUT ANY WARRANTY; see
  the LICENSE file for details.
  ------------------------------------------------------------------
  For a 1:10 scale R/C MiG-21 EDF jet.
  Board    : Classic ESP32 DevKit (ESP32-WROOM-32). Not S2/S3/C3.
  Sensor   : MPU-6050 on a GY-521 breakout
  Radio    : FlySky FS-i6X -> FS-iA10B, using the receiver's iBUS SERVO port
  Software : Arduino IDE, board package "esp32 by Espressif Systems" 3.x

  What goes through this board: CH1 aileron, CH2 elevator, CH4 rudder.
  What does NOT: CH3 throttle. The ESC plugs straight into the receiver,
  so a crash or freeze of this board can never run the fan up.

  Assumes a conventional layout: separate aileron, elevator and rudder
  servos. If your model uses elevons (delta-wing mixing), this sketch
  needs a mixer added before it is usable.

  CH5 (SwC, 3-position) = mode: low value OFF / middle RATE / high value LEVEL
  CH6 (VrA knob)        = gyro strength in RATE mode

  LED (GPIO2): solid = all good | slow blink = no radio signal |
               fast blink = sensor fault, board is in pass-through. Do not fly.

  NOT FLIGHT TESTED. It has been syntax-checked only. Do the bench tests in
  the README before the first flight and after ANY change to wiring,
  mounting or settings.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================ PINS ============================
const int PIN_IBUS_RX = 16;   // receiver iBUS SERVO port, signal wire
const int PIN_SDA     = 21;   // MPU-6050 SDA
const int PIN_SCL     = 22;   // MPU-6050 SCL
const int PIN_AIL     = 25;   // aileron servo(s), Y-lead for two
const int PIN_ELE     = 26;   // elevator servo
const int PIN_RUD     = 27;   // rudder servo
const int PIN_LED     = 2;    // onboard LED on most DevKits

// ================ RECEIVER CHANNELS (0 means CH1) ================
const int CH_AIL  = 0;
const int CH_ELE  = 1;
const int CH_RUD  = 3;
const int CH_MODE = 4;   // CH5
const int CH_GAIN = 5;   // CH6

// ============ DIRECTION SETTINGS (set during bench test) ============
// Test A1, sticks. Goal in Serial Monitor:
//   stick RIGHT -> R positive | stick PULLED BACK -> P positive | rudder RIGHT -> Y positive
const bool STICK_REV_ROLL  = false;
const bool STICK_REV_PITCH = false;
const bool STICK_REV_YAW   = false;

// Test A2, sensor. Assumes board flat, chip side up, X arrow pointing to the nose.
// Goal: right wing down -> roll and rr positive | nose up -> pitch and pr positive
//       nose swung right -> yr positive
const float ACC_ROLL_SIGN   =  1.0f;
const float ACC_PITCH_SIGN  =  1.0f;
const float GYRO_ROLL_SIGN  =  1.0f;
const float GYRO_PITCH_SIGN = -1.0f;
const float GYRO_YAW_SIGN   = -1.0f;

// Test B1, servos. Goal in OFF mode: every surface moves the right way with the sticks.
const bool SERVO_REV_AIL = false;
const bool SERVO_REV_ELE = false;
const bool SERVO_REV_RUD = false;

// Servo centre and travel in microseconds (travel = how far from centre at full stick)
const int AIL_CENTER = 1500, AIL_TRAVEL = 450;
const int ELE_CENTER = 1500, ELE_TRAVEL = 450;
const int RUD_CENTER = 1500, RUD_TRAVEL = 450;

// ============================ TUNING ============================
// RATE damping: fraction of full throw per 100 deg/s of rotation, at knob maximum.
// Start low. A fast EDF jet oscillates at high speed first.
const float DAMP_ROLL  = 0.15f;
const float DAMP_PITCH = 0.15f;
const float DAMP_YAW   = 0.15f;
// 0 = gyro always full strength, 1 = gyro fades to zero at full stick
const float STICK_PRIORITY = 0.7f;

// LEVEL mode (a "get me out of trouble" mode, not for aerobatics)
const float MAX_ROLL_DEG           = 40.0f;  // bank angle at full aileron stick
const float MAX_PITCH_DEG          = 20.0f;  // pitch angle at full elevator stick
const float LEVEL_KP_ROLL          = 0.015f; // fraction of throw per degree of error
const float LEVEL_KP_PITCH         = 0.015f;
const float LEVEL_STICK_FEED       = 0.3f;   // direct stick authority kept in LEVEL
const float LEVEL_DAMP_GAIN        = 0.5f;   // damping in LEVEL (knob ignored here)
const float LEVEL_PITCH_OFFSET_DEG = 0.0f;   // raise 1-3 if it slowly sinks in LEVEL

const bool  LEVEL_AT_BOOT = true;   // power up with the plane sitting level on its gear
const float ACC_TRUST     = 0.002f; // how fast accelerometer corrects gyro drift
const bool  DEBUG_PRINT   = true;   // set false before flying

// ======================= INTERNALS =======================
enum Mode { MODE_OFF = 0, MODE_RATE = 1, MODE_LEVEL = 2 };
const char* MODE_NAMES[] = { "OFF", "RATE", "LEVEL" };

HardwareSerial IBus(2);
uint16_t rc[14];
uint8_t  ibuf[32];
int      iidx = 0;
uint32_t lastFrameMs = 0;
bool     gotFrame = false;

const uint8_t MPU_ADDR = 0x68;
bool  imuOK = false;
int   imuErrors = 0;
float gxOff = 0, gyOff = 0, gzOff = 0;
float rollOff = 0, pitchOff = 0;
float roll = 0, pitch = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;

float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

float clamp1(float v) { return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v); }

float stickNorm(uint16_t us, bool rev) {
  float v = clamp1((us - 1500.0f) / 500.0f);
  return rev ? -v : v;
}

// ---------- iBUS: 32-byte frames, 0x20 0x40 header, 14 channels, checksum ----------
void readIBus() {
  while (IBus.available()) {
    uint8_t b = IBus.read();
    if (iidx == 0 && b != 0x20) continue;
    if (iidx == 1 && b != 0x40) { iidx = 0; continue; }
    ibuf[iidx++] = b;
    if (iidx == 32) {
      uint16_t sum = 0xFFFF;
      for (int i = 0; i < 30; i++) sum -= ibuf[i];
      uint16_t chk = ibuf[30] | (ibuf[31] << 8);
      if (sum == chk) {
        iidx = 0;
        for (int i = 0; i < 14; i++)
          rc[i] = (ibuf[2 + i * 2] | (ibuf[3 + i * 2] << 8)) & 0x0FFF;
        lastFrameMs = millis();
        gotFrame = true;
      } else {
        // Bad frame: slide forward to the next possible header instead of
        // throwing away 32 bytes, so a glitch costs one frame, not several.
        int s = 1;
        while (s < 32 && !(ibuf[s] == 0x20 && (s + 1 >= 32 || ibuf[s + 1] == 0x40))) s++;
        iidx = 32 - s;
        for (int i = 0; i < iidx; i++) ibuf[i] = ibuf[s + i];
      }
    }
  }
}

// ---------- MPU-6050 ----------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;  // not found
  mpuWrite(0x6B, 0x01);  // wake up, gyro clock
  delay(50);
  mpuWrite(0x1A, 0x03);  // internal low-pass filter ~44 Hz (fights fan vibration)
  mpuWrite(0x19, 0x00);  // 1 kHz sample rate
  mpuWrite(0x1B, 0x08);  // gyro +/-500 deg/s
  mpuWrite(0x1C, 0x08);  // accel +/-4 g
  return true;
}

bool mpuRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();   // two separate reads: order matters
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  ax = v[0] / 8192.0f; ay = v[1] / 8192.0f; az = v[2] / 8192.0f;
  gx = v[4] / 65.5f;   gy = v[5] / 65.5f;   gz = v[6] / 65.5f;
  return true;
}

float accRollDeg(float ay, float az) {
  return ACC_ROLL_SIGN * atan2f(ay, az) * RAD_TO_DEG;
}
float accPitchDeg(float ax, float ay, float az) {
  return ACC_PITCH_SIGN * atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// Plane must sit still. Returns false if it moved.
bool calibrate() {
  const int N = 500;
  float sx = 0, sy = 0, sz = 0, sr = 0, sp = 0;
  float mn[3] = { 1e9, 1e9, 1e9 }, mx[3] = { -1e9, -1e9, -1e9 };
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;
    if (!mpuRaw(ax, ay, az, gx, gy, gz)) return false;
    float g[3] = { gx, gy, gz };
    for (int k = 0; k < 3; k++) { mn[k] = fminf(mn[k], g[k]); mx[k] = fmaxf(mx[k], g[k]); }
    sx += gx; sy += gy; sz += gz;
    sr += accRollDeg(ay, az);
    sp += accPitchDeg(ax, ay, az);
    if (i % 25 == 0) digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(3);
  }
  for (int k = 0; k < 3; k++) if (mx[k] - mn[k] > 8.0f) return false;
  gxOff = sx / N; gyOff = sy / N; gzOff = sz / N;
  rollOff  = LEVEL_AT_BOOT ? sr / N : 0;
  pitchOff = LEVEL_AT_BOOT ? sp / N : 0;
  roll  = sr / N - rollOff;
  pitch = sp / N - pitchOff;
  return true;
}

// ---------- Servo output: 50 Hz, 16-bit ----------
void writeServo(int pin, float cmd, bool rev, int center, int travel) {
  cmd = clamp1(rev ? -cmd : cmd);
  int us = center + (int)(cmd * travel);
  if (us < 900) us = 900;
  if (us > 2100) us = 2100;
  ledcWrite(pin, (uint32_t)us * 65536UL / 20000UL);
}

void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);

  ledcAttach(PIN_AIL, 50, 16);
  ledcAttach(PIN_ELE, 50, 16);
  ledcAttach(PIN_RUD, 50, 16);
  writeServo(PIN_AIL, 0, false, AIL_CENTER, AIL_TRAVEL);
  writeServo(PIN_ELE, 0, false, ELE_CENTER, ELE_TRAVEL);
  writeServo(PIN_RUD, 0, false, RUD_CENTER, RUD_TRAVEL);

  IBus.begin(115200, SERIAL_8N1, PIN_IBUS_RX, -1);
  for (int i = 0; i < 14; i++) rc[i] = 1500;

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  imuOK = mpuInit();
  if (imuOK) {
    delay(500);
    bool cal = false;
    for (int t = 0; t < 5 && !cal; t++) {
      cal = calibrate();
      if (!cal) { Serial.println("Moved during calibration, hold still..."); delay(500); }
    }
    imuOK = cal;
  }
  Serial.println("MiG-21 stabilizer v1.1 (untested work in progress)");
  Serial.println(imuOK ? "Sensor OK" : "SENSOR FAULT: pass-through only. Do not fly.");
}

void loop() {
  readIBus();

  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if (nowUs - lastUs < 4000) return;          // run at 250 Hz
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;

  // ----- attitude -----
  if (imuOK) {
    float ax, ay, az, gx, gy, gz;
    if (mpuRaw(ax, ay, az, gx, gy, gz)) {
      imuErrors = 0;
      rollRate  = GYRO_ROLL_SIGN  * (gx - gxOff);
      pitchRate = GYRO_PITCH_SIGN * (gy - gyOff);
      yawRate   = GYRO_YAW_SIGN   * (gz - gzOff);
      float aR = wrap180(accRollDeg(ay, az) - rollOff);
      float aP = accPitchDeg(ax, ay, az) - pitchOff;
      float amag = sqrtf(ax * ax + ay * ay + az * az);
      float k = (fabsf(amag - 1.0f) < 0.15f) ? ACC_TRUST : 0.0f;  // ignore accel while pulling g
      float rInt = wrap180(roll + rollRate * dt);
      roll  = wrap180(rInt + k * wrap180(aR - rInt));
      float pInt = pitch + pitchRate * dt;
      pitch = pInt + k * (aP - pInt);
    } else if (++imuErrors > 20) {
      imuOK = false;   // sensor lost: fall back to pass-through
      rollRate = pitchRate = yawRate = 0;
    }
  }

  // ----- radio -----
  bool signalOK = gotFrame && (millis() - lastFrameMs < 200);
  float sR = 0, sP = 0, sY = 0, knob = 0.5f;
  Mode mode;
  if (signalOK) {
    sR = stickNorm(rc[CH_AIL], STICK_REV_ROLL);
    sP = stickNorm(rc[CH_ELE], STICK_REV_PITCH);
    sY = stickNorm(rc[CH_RUD], STICK_REV_YAW);
    knob = (rc[CH_GAIN] - 1000) / 1000.0f;
    if (knob < 0) knob = 0;
    if (knob > 1) knob = 1;
    uint16_t m = rc[CH_MODE];
    mode = (m < 1300) ? MODE_OFF : (m < 1700 ? MODE_RATE : MODE_LEVEL);
  } else if (gotFrame) {
    mode = MODE_LEVEL;   // signal lost in flight: sticks neutral, hold wings level
  } else {
    mode = MODE_OFF;     // never had signal (bench): surfaces centred
  }
  if (!imuOK) mode = MODE_OFF;

  // ----- control -----
  float oR = sR, oP = sP, oY = sY;
  if (mode == MODE_RATE) {
    oR -= DAMP_ROLL  * knob * (1 - STICK_PRIORITY * fabsf(sR)) * rollRate  / 100.0f;
    oP -= DAMP_PITCH * knob * (1 - STICK_PRIORITY * fabsf(sP)) * pitchRate / 100.0f;
    oY -= DAMP_YAW   * knob * (1 - STICK_PRIORITY * fabsf(sY)) * yawRate   / 100.0f;
  } else if (mode == MODE_LEVEL) {
    float tR = sR * MAX_ROLL_DEG;
    float tP = sP * MAX_PITCH_DEG + LEVEL_PITCH_OFFSET_DEG;
    oR = LEVEL_STICK_FEED * sR + LEVEL_KP_ROLL * wrap180(tR - roll)
         - DAMP_ROLL * LEVEL_DAMP_GAIN * rollRate / 100.0f;
    oP = LEVEL_STICK_FEED * sP + LEVEL_KP_PITCH * (tP - pitch)
         - DAMP_PITCH * LEVEL_DAMP_GAIN * pitchRate / 100.0f;
    oY = sY - DAMP_YAW * LEVEL_DAMP_GAIN * (1 - STICK_PRIORITY * fabsf(sY)) * yawRate / 100.0f;
  }

  writeServo(PIN_AIL, oR, SERVO_REV_AIL, AIL_CENTER, AIL_TRAVEL);
  writeServo(PIN_ELE, oP, SERVO_REV_ELE, ELE_CENTER, ELE_TRAVEL);
  writeServo(PIN_RUD, oY, SERVO_REV_RUD, RUD_CENTER, RUD_TRAVEL);

  // ----- LED -----
  uint32_t ms = millis();
  if (!imuOK)         digitalWrite(PIN_LED, (ms / 100) % 2);
  else if (!signalOK) digitalWrite(PIN_LED, (ms / 500) % 2);
  else                digitalWrite(PIN_LED, HIGH);

  // ----- debug -----
  static uint32_t lastPrint = 0;
  if (DEBUG_PRINT && ms - lastPrint > 100) {
    lastPrint = ms;
    Serial.printf("%-5s sig:%d imu:%d | stick R:%+.2f P:%+.2f Y:%+.2f knob:%.2f | "
                  "roll:%+6.1f pitch:%+6.1f | rr:%+6.1f pr:%+6.1f yr:%+6.1f | out R:%+.2f P:%+.2f Y:%+.2f\n",
                  MODE_NAMES[mode], signalOK, imuOK, sR, sP, sY, knob,
                  roll, pitch, rollRate, pitchRate, yawRate, oR, oP, oY);
  }
}
