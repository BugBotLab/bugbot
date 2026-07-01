# Adafruit NeoPixel â€” clone from:
# https://github.com/adafruit/Adafruit_NeoPixel.git
cmake_minimum_required(VERSION 3.16)

file(GLOB SRCS "*.cpp" "src/*.cpp")
list(APPEND SRCS "esp.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32 driver
)
