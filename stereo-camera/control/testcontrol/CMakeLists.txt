cmake_minimum_required(VERSION 3.14)
project(test_servo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_servo
    test_servo.cpp
    ../ServoClient.cpp
)

target_include_directories(test_servo PRIVATE ..)
target_link_libraries(test_servo PRIVATE pthread)
