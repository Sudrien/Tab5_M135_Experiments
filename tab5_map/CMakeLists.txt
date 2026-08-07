# ESP-IDF project definition.
#
# UNTESTED. I have no IDF toolchain here, so treat this as a starting point
# rather than something known to build. The parts I did verify are noted in
# OPTIMIZATION_NOTES.md.
#
# arduino-cli ignores this file, along with sdkconfig.defaults and main/ - it
# compiles the sketch folder and src/ only. So adding these does not disturb
# the Arduino build.
#
# Prerequisites:
#   - arduino-esp32 available as an IDF component (components/arduino, or via
#     the component manager). The Arduino APIs this project uses - WiFi,
#     HTTPClient, SD_MMC, FS - all come from there, so nothing in the sources
#     has to change.
#   - M5Unified and M5GFX likewise. Both support IDF component builds.
#
# Build:  idf.py set-target esp32p4 && idf.py build

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tab5_map)
