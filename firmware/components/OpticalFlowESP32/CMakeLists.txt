# OpticalFlowESP32 â€” clone from your project's repository (private/custom library).
# Update the URL in bootstrap.ps1 before running it.
cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c" "*.cpp" "*.c")
list(FILTER SRCS EXCLUDE REGEX ".*(examples|test|tests)/.*")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src" "."
    REQUIRES espressif__arduino-esp32 driver
)

target_compile_options(__idf_OpticalFlowESP32 PRIVATE -Wno-reorder)
