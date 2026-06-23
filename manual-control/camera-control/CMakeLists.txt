cmake_minimum_required(VERSION 3.10)
project(camera_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_executable(camera_control
    main.cpp
    serial_link.cpp
)

target_compile_options(camera_control PRIVATE -Wall -Wextra -O2)
target_link_libraries(camera_control PRIVATE Threads::Threads)
