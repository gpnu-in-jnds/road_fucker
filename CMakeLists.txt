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

add_library(road_fucker_lib
    src/road_detector.cpp
    src/camera.cpp
    src/preview.cpp
    src/uart_sender.cpp)
target_include_directories(road_fucker_lib PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(road_fucker_lib PUBLIC ${OpenCV_LIBS})
target_compile_options(road_fucker_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(road_fucker src/main.cpp)
target_link_libraries(road_fucker PRIVATE road_fucker_lib)
target_compile_options(road_fucker PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(road_fucker_tests tests/road_detector_test.cpp)
  target_link_libraries(road_fucker_tests PRIVATE road_fucker_lib)
  target_compile_definitions(road_fucker_tests PRIVATE
    ROAD_FUCKER_SAMPLE_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/frame_784.png"
    ROAD_FUCKER_CLOSE_T_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/t_junction_close.png"
    ROAD_FUCKER_NEAR_T_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/t_junction_near_raw.png"
    ROAD_FUCKER_APPROACH_T_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/t_junction_approach_raw.png"
    ROAD_FUCKER_LIVE_T_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/t_junction_live_720p.png")
  add_test(NAME road_fucker_tests COMMAND road_fucker_tests)

  add_executable(road_fucker_preview_tests tests/preview_window_test.cpp)
  target_link_libraries(road_fucker_preview_tests PRIVATE road_fucker_lib)
  target_compile_options(road_fucker_preview_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_fucker_preview_tests COMMAND road_fucker_preview_tests)

  add_executable(road_fucker_source_tests tests/source_test.cpp)
  target_link_libraries(road_fucker_source_tests PRIVATE road_fucker_lib)
  target_compile_options(road_fucker_source_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_fucker_source_tests COMMAND road_fucker_source_tests)

  add_executable(road_fucker_config_tests tests/config_test.cpp)
  target_link_libraries(road_fucker_config_tests PRIVATE road_fucker_lib)
  target_compile_options(road_fucker_config_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_fucker_config_tests COMMAND road_fucker_config_tests)

  add_executable(road_fucker_uart_tests tests/uart_packet_test.cpp)
  target_link_libraries(road_fucker_uart_tests PRIVATE road_fucker_lib)
  target_compile_definitions(road_fucker_uart_tests PRIVATE
    ROAD_FUCKER_NEAR_T_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/t_junction_near_raw.png")
  target_compile_options(road_fucker_uart_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_fucker_uart_tests COMMAND road_fucker_uart_tests)
endif()
