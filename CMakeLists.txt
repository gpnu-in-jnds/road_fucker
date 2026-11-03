cmake_minimum_required(VERSION 3.16)
project(road_fucker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV 4 QUIET COMPONENTS core imgproc videoio highgui imgcodecs)
if(NOT OpenCV_FOUND)
  # OpenCV 5 is accepted for development machines; the target platform uses 4.
  find_package(OpenCV 5 REQUIRED COMPONENTS core geometry imgproc videoio highgui imgcodecs)
endif()

find_package(Threads REQUIRED)

add_library(road_fucker_lib
    src/road_detector.cpp
    src/occlusion.cpp
    src/worker_pool.cpp
    src/camera.cpp
    src/preview.cpp
    src/uart_sender.cpp)
target_include_directories(road_fucker_lib PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(road_fucker_lib PUBLIC ${OpenCV_LIBS} Threads::Threads)
target_compile_options(road_fucker_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(road_fucker src/main.cpp)
target_link_libraries(road_fucker PRIVATE road_fucker_lib)
target_compile_options(road_fucker PRIVATE -Wall -Wextra -Wpedantic)

option(BUILD_TESTING "Build tests" OFF)
include(CTest)
if(BUILD_TESTING)
  foreach(feature IN ITEMS road_detector field_conditions parallel_detection preview_window source config uart_packet)
    add_executable(${feature}_test tests/${feature}_test.cpp)
    target_link_libraries(${feature}_test PRIVATE road_fucker_lib)
    target_compile_options(${feature}_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME ${feature}_test COMMAND ${feature}_test)
  endforeach()
  target_compile_definitions(parallel_detection_test PRIVATE
    ROAD_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
  add_executable(headless_cli_fixture tests/headless_cli_fixture.cpp)
  target_link_libraries(headless_cli_fixture PRIVATE road_fucker_lib)
  target_compile_options(headless_cli_fixture PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_fucker_headless_cli COMMAND ${CMAKE_COMMAND}
    -DAPP=$<TARGET_FILE:road_fucker>
    -DFIXTURE=$<TARGET_FILE:headless_cli_fixture>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/headless_cli_test.cmake)
  set_tests_properties(road_fucker_headless_cli PROPERTIES
    ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY=" TIMEOUT 30)

else()
  # CMake can leave a previous ON configuration's registration file behind.
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/CTestTestfile.cmake"
    "# Tests disabled by BUILD_TESTING=OFF.\n")
endif()
