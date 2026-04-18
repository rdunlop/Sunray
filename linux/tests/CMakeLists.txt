cmake_minimum_required(VERSION 3.14)

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.5.4
)
FetchContent_MakeAvailable(Catch2)

# wiring_main.cpp compiled with NO_MAIN so millis()/micros() are available
# without emitting a competing main() symbol.
set(WIRING_MAIN_SRC ${CMAKE_SOURCE_DIR}/src/wiring_main.cpp)

add_executable(test_sunray
  ${WIRING_MAIN_SRC}
  test_line_tracker.cpp
)
target_compile_definitions(test_sunray PRIVATE NO_MAIN)
target_link_libraries(test_sunray PRIVATE sunray_lib Catch2::Catch2WithMain)
target_include_directories(test_sunray PRIVATE
  ${CMAKE_SOURCE_DIR}/src        # Arduino.h, HardwareSerial.h, etc.
  ${FIRMWARE_PATH}               # robot.h, LineTracker.h, map.h, etc.
  ${FIRMWARE_PATH}/src           # types.h, op/op.h, etc.
  ${CMAKE_CURRENT_SOURCE_DIR}/support
)

include(CTest)
# catch_discover_tests is avoided here: the firmware's background threads (serial
# ports, BLE) print to stdout during binary enumeration, causing ctest to register
# each output line as a test name.  A single add_test entry runs the full Catch2
# suite; Catch2 still reports each TEST_CASE individually in its own output.
add_test(NAME sunray_unit_tests COMMAND test_sunray)
