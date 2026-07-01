# ArucoLite (AprilTag detection) â€” clone from your project's repository.
# Update the URL in bootstrap.ps1 before running it.
cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c" "src/*.cc" "*.cpp" "*.c")
list(FILTER SRCS EXCLUDE REGEX ".*(examples|test|tests)/.*")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src" "."
    REQUIRES espressif__arduino-esp32
)
