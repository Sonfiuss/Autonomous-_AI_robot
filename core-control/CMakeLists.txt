cmake_minimum_required(VERSION 3.16)
project(core_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(core_control
    main.cpp
    KinematicsCore.cpp
    CommandParser.cpp
)

target_include_directories(core_control PRIVATE ${CMAKE_SOURCE_DIR})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(core_control PRIVATE -O3 -march=native)
endif()

install(TARGETS core_control RUNTIME DESTINATION bin)
