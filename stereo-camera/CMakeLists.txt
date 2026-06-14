cmake_minimum_required(VERSION 3.16)
project(stereo_scan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc calib3d)
find_package(Threads REQUIRED)

# ── Pipeline static library ───────────────────────────────────────────────────
# All pipeline + vision code in one library so executables and tests share it.
add_library(stereo_pipeline STATIC
    control/ServoController.cpp
    control/ServoClient.cpp
    vision/StereoCamera.cpp
    vision/MapBuilder.cpp
    pipeline/Scanner.cpp
    # ── core fusion (port of implement_plan_stereo.md) ──
    core/Geometry.cpp
    core/AngleBuffer.cpp
    core/Depth.cpp
    core/Reconstruct.cpp
    core/Sync.cpp
    calibration/Calibration.cpp
)

target_include_directories(stereo_pipeline PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(stereo_pipeline PUBLIC
    ${OpenCV_LIBS}
    Threads::Threads
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(stereo_pipeline PRIVATE -O3 -march=native)
endif()

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(stereo_scan main.cpp)
target_link_libraries(stereo_scan PRIVATE stereo_pipeline)

# ── Servo test tool — velocity mode (old ServoClient protocol) ───────────────
add_executable(servo_test
    tools/servo_test.cpp
    control/ServoClient.cpp
)
target_include_directories(servo_test PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(servo_test PRIVATE Threads::Threads)

# ── servo_ctrl — debug tool for MOVE/OK step-and-hold protocol ───────────────
add_executable(servo_ctrl
    tools/servo_ctrl.cpp
    control/ServoController.cpp
)
target_include_directories(servo_ctrl PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(servo_ctrl PRIVATE Threads::Threads)

# ── Unit tests ────────────────────────────────────────────────────────────────
enable_testing()

# test_frame_queue — no OpenCV, only Threads
add_executable(test_frame_queue tests/test_frame_queue.cpp)
target_include_directories(test_frame_queue PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(test_frame_queue PRIVATE Threads::Threads)
add_test(NAME frame_queue COMMAND test_frame_queue)

# test_scanner_raster — needs Scanner (which pulls in OpenCV via stereo_pipeline)
add_executable(test_scanner_raster tests/test_scanner_raster.cpp)
target_link_libraries(test_scanner_raster PRIVATE stereo_pipeline)
add_test(NAME scanner_raster COMMAND test_scanner_raster)

# test_map_builder — needs OpenCV
add_executable(test_map_builder tests/test_map_builder.cpp)
target_link_libraries(test_map_builder PRIVATE stereo_pipeline)
add_test(NAME map_builder COMMAND test_map_builder)

# ── core fusion tests (plan §8 validation tables) ─────────────────────────────
foreach(t geometry angle_buffer depth reconstruct accumulate end_to_end sync calibration)
    add_executable(test_${t} tests/test_${t}.cpp)
    target_link_libraries(test_${t} PRIVATE stereo_pipeline)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS stereo_scan servo_test RUNTIME DESTINATION bin)
