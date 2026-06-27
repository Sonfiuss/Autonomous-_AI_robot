cmake_minimum_required(VERSION 3.16)
project(adcensus_depth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV is used ONLY for image I/O (imread/imwrite) and the JET colormap.
# The AD-Census algorithm itself is library-independent.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

# Vendored AD-Census algorithm (unmodified upstream sources).
add_library(adcensus STATIC
    adcensus/ADCensusStereo.cpp
    adcensus/adcensus_util.cpp
    adcensus/cost_computor.cpp
    adcensus/cross_aggregator.cpp
    adcensus/scanline_optimizer.cpp
    adcensus/multistep_refiner.cpp
)
target_include_directories(adcensus PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
# Upstream comments are GBK-encoded; silence GCC's invalid-UTF-8 noise.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(adcensus PRIVATE -finput-charset=UTF-8 -w)
endif()

# Jetson (aarch64) optimization, mirroring the main stereo-camera module.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(adcensus PRIVATE -O3 -march=native)
endif()

add_executable(adcensus_depth main.cpp)
target_link_libraries(adcensus_depth PRIVATE adcensus ${OpenCV_LIBS})
target_include_directories(adcensus_depth PRIVATE ${OpenCV_INCLUDE_DIRS})
