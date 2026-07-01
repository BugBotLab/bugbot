// ServoLib.h
// Dual PWM servo driver using the ESP32Servo library. Controls two servos
// independently by degree (0–180), defaulting to 90° on begin().
#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class ServoLib {
public:
  void begin(int pin1, int pin2) {
    _pin1 = pin1;
    _pin2 = pin2;

    _servo1.attach(_pin1, 500, 2500);
    _servo2.attach(_pin2, 500, 2500);

    setServo1Deg(90.0f);
    setServo2Deg(90.0f);
  }

  void setServo1Deg(float deg) {
    _servo1.write(constrain((int)deg, 0, 180));
  }

  void setServo2Deg(float deg) {
    _servo2.write(constrain((int)deg, 0, 180));
  }

  void detachAll() {
    if (_servo1.attached()) _servo1.detach();
    if (_servo2.attached()) _servo2.detach();
  }

private:
  int _pin1 = -1;
  int _pin2 = -1;
  Servo _servo1;
  Servo _servo2;
};