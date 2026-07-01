# AsyncTCP â€” clone from: https://github.com/me-no-dev/AsyncTCP.git
cmake_minimum_required(VERSION 3.16)

file(GLOB SRCS "src/*.cpp")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES espressif__arduino-esp32 esp_wifi
)
