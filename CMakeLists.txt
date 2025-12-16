cmake_minimum_required(VERSION 3.13)
#set(PICO_BOARD pico)
#set(PICO_BOARD pico_w)
set(PICO_BOARD pico2)

# If pico2 is selected you have to select PICO_PLATFORM
set(PICO_PLATFORM rp2350-arm-s)
# set(PICO_PLATFORM rp2350-riscv)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(ism330dlc_driver_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(driver)
add_subdirectory(platform/pico)
add_subdirectory(examples/pico/example_1)