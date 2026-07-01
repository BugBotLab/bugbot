# espressif__arduino-esp32 MIDI Library â€” clone from:
# https://github.com/FortySevenEffects/arduino_midi_library.git
cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES espressif__arduino-esp32
)
