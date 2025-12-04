#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int ch1Pin = 35;
const int ch2Pin = 34;
const int ch5Pin = 27;
const int ch6Pin = 14;

const int leftRPWM = 26;
const int leftLPWM = 25;
const int rightRPWM = 33;
const int rightLPWM = 32;

const int pwmFreq = 20000;
const int pwmResBits = 8;
const int maxDuty = 255;

enum SpeedMode { CRAWLER, NORMAL, TURBO };

unsigned long lastDebugTime = 0;
unsigned long debugInterval = 100;

bool launchActive = false;
unsigned long launchStart = 0;
bool normalSeenSinceLastLaunch = true;

float lastCh1Norm = 0.0f;
float lastCh2Norm = 0.0f;

unsigned long readPulseSafe(int pin) {
  unsigned long p = pulseIn(pin, HIGH, 25000);
  return p;
}

float normRC(unsigned long us, float lastVal) {
  if (us < 900 || us > 2100) return lastVal;
  float v = ((float)us - 1500.0f) / 500.0f;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return v;
}

SpeedMode decodeSpeedMode(unsigned long ch6us) {
  if (ch6us < 1300) return CRAWLER;
  if (ch6us > 1700) return TURBO;
  return NORMAL;
}

bool decodeTraction(unsigned long ch5us) {
  return ch5us > 1500;
}

void setupPwmPin(int pin) {
  analogWriteResolution(pin, pwmResBits);
  analogWriteFrequency(pin, pwmFreq);
  analogWrite(pin, 0);
}

void setMotor(int pinFwd, int pinRev, float value) {
  if (value > 1.0f) value = 1.0f;
  if (value < -1.0f) value = -1.0f;

  int duty = 0;

  if (value > 0.02f) {
    duty = (int)(value * maxDuty);
    if (duty > maxDuty) duty = maxDuty;
    if (duty < 0) duty = 0;
    analogWrite(pinFwd, duty);
    analogWrite(pinRev, 0);
    digitalWrite(pinRev, LOW);
  } else if (value < -0.02f) {
    duty = (int)(-value * maxDuty);
    if (duty > maxDuty) duty = maxDuty;
    if (duty < 0) duty = 0;
    analogWrite(pinRev, duty);
    analogWrite(pinFwd, 0);
    digitalWrite(pinFwd, LOW);
  } else {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, 0);
    digitalWrite(pinFwd, LOW);
    digitalWrite(pinRev, LOW);
  }
}

void setup() {
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch5Pin, INPUT);
  pinMode(ch6Pin, INPUT);

  pinMode(leftRPWM, OUTPUT);
  pinMode(leftLPWM, OUTPUT);
  pinMode(rightRPWM, OUTPUT);
  pinMode(rightLPWM, OUTPUT);

  setupPwmPin(leftRPWM);
  setupPwmPin(leftLPWM);
  setupPwmPin(rightRPWM);
  setupPwmPin(rightLPWM);

  Serial.begin(115200);
  SerialBT.begin("BlazeBot");
}

void loop() {
  unsigned long ch1us = readPulseSafe(ch1Pin);
  unsigned long ch2us = readPulseSafe(ch2Pin);
  unsigned long ch5us = readPulseSafe(ch5Pin);
  unsigned long ch6us = readPulseSafe(ch6Pin);

  lastCh1Norm = normRC(ch1us, lastCh1Norm);
  lastCh2Norm = normRC(ch2us, lastCh2Norm);

  float steering = lastCh1Norm;
  float throttle = lastCh2Norm;

  SpeedMode mode = decodeSpeedMode(ch6us);
  bool traction = decodeTraction(ch5us);

  if (mode == NORMAL) {
    normalSeenSinceLastLaunch = true;
  }

  if (traction && mode == TURBO && !launchActive && normalSeenSinceLastLaunch) {
    launchActive = true;
    launchStart = millis();
    normalSeenSinceLastLaunch = false;
  }

  if (launchActive) {
    if (millis() - launchStart > 700) {
      launchActive = false;
    } else {
      float boost = 1.2f;
      throttle *= boost;
      if (throttle > 1.0f) throttle = 1.0f;
      if (throttle < -1.0f) throttle = -1.0f;
    }
  }

  float speedScale = 1.0f;
  if (mode == CRAWLER) speedScale = 0.35f;
  else if (mode == NORMAL) speedScale = 0.7f;
  else speedScale = 1.0f;

  throttle *= speedScale;

  float left = throttle + steering;
  float right = throttle - steering;

  float maxMag = fmax(fabs(left), fabs(right));
  if (maxMag > 1.0f) {
    left /= maxMag;
    right /= maxMag;
  }

  setMotor(leftRPWM, leftLPWM, left);
  setMotor(rightRPWM, rightLPWM, right);

  unsigned long now = millis();
  if (now - lastDebugTime > debugInterval) {
    lastDebugTime = now;

    SerialBT.print("CH1=");
    SerialBT.print(ch1us);
    SerialBT.print(" CH2=");
    SerialBT.print(ch2us);
    SerialBT.print(" CH5=");
    SerialBT.print(ch5us);
    SerialBT.print(" CH6=");
    SerialBT.print(ch6us);

    SerialBT.print(" steer=");
    SerialBT.print(steering, 2);
    SerialBT.print(" thr=");
    SerialBT.print(throttle, 2);

    SerialBT.print(" L=");
    SerialBT.print(left, 2);
    SerialBT.print(" R=");
    SerialBT.print(right, 2);

    SerialBT.print(" mode=");
    if (mode == CRAWLER) SerialBT.print("CRAWLER");
    else if (mode == NORMAL) SerialBT.print("NORMAL");
    else SerialBT.print("TURBO");

    SerialBT.print(" traction=");
    SerialBT.print(traction ? "ON" : "OFF");

    SerialBT.print(" launch=");
    SerialBT.println(launchActive ? "ON" : "OFF");
  }
}
