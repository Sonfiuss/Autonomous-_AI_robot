# ESP-IDF project for the unified controller.
#   idf.py set-target esp32
#   idf.py build flash monitor
#
# UNVERIFIED: no ESP-IDF toolchain exists on the machine this was written on.
cmake_minimum_required(VERSION 3.16)

# project/ registers itself as an IDF component (see its CMakeLists.txt), so RM,
# MC and LINK are compiled into the firmware from their single source of truth
# instead of being copied in.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../project")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_unified_controller)
