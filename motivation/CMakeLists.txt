cmake_minimum_required(VERSION 3.16)
project(motivation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_executable(motivation
    main.cpp
    MotionClient.cpp
    SimBridge.cpp
)

target_include_directories(motivation PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${ZMQ_INCLUDE_DIRS}
)

target_link_libraries(motivation PRIVATE
    Threads::Threads
    ${ZMQ_LIBRARIES}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(motivation PRIVATE -O3 -march=native)
endif()

install(TARGETS motivation RUNTIME DESTINATION bin)
