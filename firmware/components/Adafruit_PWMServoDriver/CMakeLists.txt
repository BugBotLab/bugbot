# Adafruit PWM Servo Driver (PCA9685) â€” clone from:
# https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.git
cmake_minimum_required(VERSION 3.16)

# Library ships with flat layout (headers/sources in root) OR src/ â€” handle both.
file(GLOB SRCS "*.cpp" "src/*.cpp")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32 Adafruit_BusIO
)
