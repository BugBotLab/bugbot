# SparkFun BMI270 Library â€” clone from:
# https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library.git
cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES espressif__arduino-esp32
)
