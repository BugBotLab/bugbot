# ESP32Servo â€” clone from: https://github.com/madhephaestus/ESP32Servo.git
cmake_minimum_required(VERSION 3.16)

file(GLOB SRCS "src/*.cpp" "*.cpp")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src" "."
    REQUIRES espressif__arduino-esp32 driver
)
