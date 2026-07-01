// RobotConfig.h
// Static hardware configuration: pin assignments and low-level polling frequencies.
// Edit this file when changing hardware layout; runtime tunables live in AppConfig.h.
#pragma once
#include <Arduino.h>

struct RobotPins {
  // I2C
  int i2c_sda = 5;
  int i2c_scl = 6;

  // Board / power
  int sys_enable = 43;
  int usb_detect = 1;
  int bat_adc    = 3;
  int sleep_toggle = 2;
  int int_in     = 44;

  // New actuator pins on this board revision
  int rgb_led_data = 4;   // WS2812 / NeoPixel-style serial LED
  int buzzer_pwm   = 7;
  int servo2_pwm   = 8;
  int servo1_pwm   = 9;
};

struct RobotConfig {
  uint32_t i2c_hz        = 400000;
  uint16_t i2c_timeoutms = 100;
  int      odom_hz       = 200;
  int      diag_hz       = 10;
  int      udp_hz        = 50;
  int      lidar_row     = 1;
  int      lidar_hz      = 30;
  int      low_power_poll_ms = 250;
  int      low_power_cpu_mhz = 80;

  RobotPins pins{};
};

inline RobotConfig makeDefaultRobotConfig() {
  return RobotConfig{};
}
