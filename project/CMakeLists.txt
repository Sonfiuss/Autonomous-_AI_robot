cmake_minimum_required(VERSION 3.13)

set(RM_SOURCES
    src/RM/omni_kinematics.cpp
    src/RM/velocity_profile.cpp
    src/RM/speed_limit.cpp
    src/RM/odometry.cpp
    src/RM/driver_stepdir.cpp)

set(MV_SOURCES
    src/MV/occupancy_grid.cpp
    src/MV/astar.cpp
    src/MV/path.cpp
    src/MV/mv_api.cpp)

set(MC_SOURCES
    src/MC/executor.cpp
    src/MC/mc_api.cpp)

# The Jetson <-> ESP32 wire protocol. Built into both sides of the link, which
# is the point: one implementation, so the two can never disagree about a line.
set(LINK_SOURCES
    src/LINK/protocol.cpp)

# The Jetson-side plan sequencer. NOT registered with ESP-IDF below: the firmware
# executes legs, it does not sequence them, and giving it this code would only
# invite the two to disagree about whose job that is.
set(SEQ_SOURCES
    src/SEQ/sequencer.cpp)

# --------------------------------------------------------------------------
# ESP-IDF: when this directory sits under <firmware>/components/ the IDF build
# system includes this file and idf_component_register is defined.
# --------------------------------------------------------------------------
if(COMMAND idf_component_register)
    idf_component_register(SRCS ${RM_SOURCES} ${MV_SOURCES} ${MC_SOURCES} ${LINK_SOURCES}
                           INCLUDE_DIRS include config)
    # C++14, like the PC build below. IDF's default is gnu++2b, whose <atomic>
    # pulls in unistd.h, and its `int link()` collides with `namespace link`.
    set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 14)
    return()
endif()

# --------------------------------------------------------------------------
# PC build: static libraries, MV shared library (for Python ctypes), CLI, unit tests.
#   mkdir build && cd build && cmake .. && cmake --build . && ctest
# Without CMake: tools/build_mv.sh (MV only) or the g++ one-liners in README.md.
# --------------------------------------------------------------------------
project(robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RM_BUILD_TESTS "Build RM unit tests" ON)
option(RM_DEBUG "Enable RM_DLOG debug output" OFF)
option(MV_BUILD_TESTS "Build MV unit tests and CLI" ON)
option(MV_DEBUG "Enable MV_DLOG debug output" OFF)
option(MC_BUILD_TESTS "Build MC unit tests and CLI" ON)
option(MC_DEBUG "Enable MC_DLOG debug output" OFF)
option(LINK_BUILD_TESTS "Build LINK unit tests" ON)
option(LINK_DEBUG "Enable LINK_DLOG debug output" OFF)
option(SEQ_BUILD_TESTS "Build SEQ unit tests" ON)
option(SEQ_DEBUG "Enable SEQ_DLOG debug output" OFF)

add_library(rm STATIC ${RM_SOURCES})
target_include_directories(rm PUBLIC include config)
if(RM_DEBUG)
    target_compile_definitions(rm PUBLIC RM_DEBUG)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(rm PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
endif()

if(RM_BUILD_TESTS)
    enable_testing()
    add_executable(test_rm tests/test_rm.cpp)
    target_link_libraries(test_rm PRIVATE rm)
    add_test(NAME rm_unit COMMAND test_rm)
endif()

# ---- MV: static lib (firmware / C++ callers) + shared lib (ctypes from project/src/CM)
add_library(mv STATIC ${MV_SOURCES})
target_include_directories(mv PUBLIC include config)
add_library(mv_shared SHARED ${MV_SOURCES})
target_include_directories(mv_shared PUBLIC include config)
target_compile_definitions(mv_shared PRIVATE MV_BUILD_SHARED)
set_target_properties(mv_shared PROPERTIES OUTPUT_NAME mv PREFIX "")   # mv.dll / mv.so
if(MV_DEBUG)
    target_compile_definitions(mv PUBLIC MV_DEBUG)
    target_compile_definitions(mv_shared PUBLIC MV_DEBUG)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mv PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
    target_compile_options(mv_shared PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
    if(WIN32)
        target_link_options(mv_shared PRIVATE -static-libgcc -static-libstdc++)
    endif()
endif()

if(MV_BUILD_TESTS)
    enable_testing()
    add_executable(mv_cli tools/mv_cli.cpp)
    target_link_libraries(mv_cli PRIVATE mv)
    add_executable(test_mv tests/test_mv.cpp)
    target_link_libraries(test_mv PRIVATE mv)
    add_test(NAME mv_unit COMMAND test_mv)
endif()

# ---- MC: motion executor. Links RM (all its maths) and, for the end-to-end
# test only, MV (it plans the route the executor then drives).
add_library(mc STATIC ${MC_SOURCES})
target_include_directories(mc PUBLIC include config)
target_link_libraries(mc PUBLIC rm)
add_library(mc_shared SHARED ${MC_SOURCES} ${RM_SOURCES})
target_include_directories(mc_shared PUBLIC include config)
target_compile_definitions(mc_shared PRIVATE MC_BUILD_SHARED)
set_target_properties(mc_shared PROPERTIES OUTPUT_NAME mc PREFIX "")   # mc.dll / mc.so
if(MC_DEBUG)
    target_compile_definitions(mc PUBLIC MC_DEBUG)
    target_compile_definitions(mc_shared PUBLIC MC_DEBUG)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mc PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
    target_compile_options(mc_shared PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
    if(WIN32)
        target_link_options(mc_shared PRIVATE -static-libgcc -static-libstdc++)
    endif()
endif()

if(MC_BUILD_TESTS)
    enable_testing()
    add_executable(mc_cli tools/mc_cli.cpp)
    target_link_libraries(mc_cli PRIVATE mc)
    add_executable(test_mc tests/test_mc.cpp)
    target_link_libraries(test_mc PRIVATE mc mv)
    add_test(NAME mc_unit COMMAND test_mc)
endif()

# ---- LINK: the wire protocol. Depends on config/constants.h and nothing else,
# so both the ESP32 firmware and the Jetson application can link it as-is.
add_library(link STATIC ${LINK_SOURCES})
target_include_directories(link PUBLIC include config)
if(LINK_DEBUG)
    target_compile_definitions(link PUBLIC LINK_DEBUG)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(link PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
endif()

if(LINK_BUILD_TESTS)
    enable_testing()
    add_executable(test_link tests/test_link.cpp)
    target_link_libraries(test_link PRIVATE link)
    add_test(NAME link_unit COMMAND test_link)
endif()

# ---- SEQ: walks an MV primitive list onto the wire, one leg at a time. Links RM
# (it estimates each leg's duration the way the firmware plans it) and LINK (it
# emits wire commands). Only MC/types.h is used from MC, which is header-only.
add_library(seq STATIC ${SEQ_SOURCES})
target_include_directories(seq PUBLIC include config)
target_link_libraries(seq PUBLIC rm link)
if(SEQ_DEBUG)
    target_compile_definitions(seq PUBLIC SEQ_DEBUG)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(seq PRIVATE -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti)
endif()

if(SEQ_BUILD_TESTS)
    enable_testing()
    add_executable(test_seq tests/test_seq.cpp)
    target_link_libraries(test_seq PRIVATE seq)
    add_test(NAME seq_unit COMMAND test_seq)
endif()
