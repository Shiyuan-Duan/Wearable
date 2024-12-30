cmake_minimum_required(VERSION 3.20.0)
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/max30003
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/max86141
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ads1299
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/W25M02GW
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/afe4960
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/max30001
  # ${CMAKE_CURRENT_BINARY_DIR}/zephyr/include/generated/syscalls

  # ${CMAKE_CURRENT_SOURCE_DIR}/lib/tflite-micro
  
)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Wearable)
# target_link_libraries(app PUBLIC tensorflow_lite_micro)
set(NO_THREADSAFE_STATICS $<TARGET_PROPERTY:compiler-cpp,no_threadsafe_statics>)
zephyr_compile_options($<$<COMPILE_LANGUAGE:CXX>:${NO_THREADSAFE_STATICS}>)


add_subdirectory(src/ble)
add_subdirectory(src/sensors)


file(GLOB SRC_FILES "src/*.c")

target_sources(app PRIVATE ${SRC_FILES})

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ble
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors
)
