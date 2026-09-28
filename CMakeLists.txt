cmake_minimum_required(VERSION 3.16)
project(road_finder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 QUIET COMPONENTS core imgproc videoio highgui imgcodecs)
if(NOT OpenCV_FOUND)
  # OpenCV 5 is accepted for development machines; the target platform uses 4.
  find_package(OpenCV 5 REQUIRED COMPONENTS core geometry imgproc videoio highgui imgcodecs)
endif()

add_library(road_finder_lib src/road_detector.cpp src/camera.cpp src/preview.cpp)
target_include_directories(road_finder_lib PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(road_finder_lib PUBLIC ${OpenCV_LIBS})
target_compile_options(road_finder_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(road_finder src/main.cpp)
target_link_libraries(road_finder PRIVATE road_finder_lib)
target_compile_options(road_finder PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(road_finder_tests tests/road_detector_test.cpp)
  target_link_libraries(road_finder_tests PRIVATE road_finder_lib)
  target_compile_definitions(road_finder_tests PRIVATE
    ROAD_FINDER_SAMPLE_IMAGE="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/frame_784.png")
  add_test(NAME road_finder_tests COMMAND road_finder_tests)

  add_executable(road_finder_preview_tests tests/preview_window_test.cpp)
  target_link_libraries(road_finder_preview_tests PRIVATE road_finder_lib)
  target_compile_options(road_finder_preview_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_finder_preview_tests COMMAND road_finder_preview_tests)

  add_executable(road_finder_source_tests tests/source_test.cpp)
  target_link_libraries(road_finder_source_tests PRIVATE road_finder_lib)
  target_compile_options(road_finder_source_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_finder_source_tests COMMAND road_finder_source_tests)

  add_executable(road_finder_config_tests tests/config_test.cpp)
  target_link_libraries(road_finder_config_tests PRIVATE road_finder_lib)
  target_compile_options(road_finder_config_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME road_finder_config_tests COMMAND road_finder_config_tests)
endif()
