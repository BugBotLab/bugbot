cmake_minimum_required(VERSION 3.16)

file(GLOB SRCS "*.cpp")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32
)
