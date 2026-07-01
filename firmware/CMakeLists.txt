cmake_minimum_required(VERSION 3.16)
# espressif__arduino-esp32-esp32 is declared as a managed component in main/idf_component.yml.
# Run `idf.py update-dependencies` once after cloning to fetch it.


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BugBot_Firmware)
