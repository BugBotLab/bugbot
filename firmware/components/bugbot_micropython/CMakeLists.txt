# BugBot MicroPython integration scaffold.
#
# Full integration requires building MicroPython as an ESP-IDF component:
#   git clone https://github.com/micropython/micropython.git
#   Follow: micropython/ports/esp32/README.md
#
# This scaffold compiles today with a no-op placeholder so the rest of the
# firmware builds. Replace bugbot_module.c with the real implementation once
# the micropython component is added to this project.
cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS "bugbot_module.c"
    INCLUDE_DIRS "."
    REQUIRES pocketpy espressif__arduino-esp32
)
