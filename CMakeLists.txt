cmake_minimum_required(VERSION 3.21)

option(BUILD_STANDALONE "Initialize the Pico SDK in the library"                  OFF)
option(ISM330DLC_BUILD_COMPLEMENTARY_FILTER_EXAMPLE "Build sensor fusion example" OFF)
option(ISM330DLC_BUILD_PAL_TEST "Build PAL test example"                          OFF)

# If standalone, set up the Raspberry Pi Pico SDK
if (BUILD_STANDALONE)
    set(PICO_BOARD pico2)
    set(PICO_PLATFORM rp2350-arm-s)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    set(CMAKE_COLOR_DIAGNOSTICS ON)
    include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    project(pico_ism330dlc         
        VERSION 1.0.0
        LANGUAGES C CXX ASM
    )
    pico_sdk_init()
else()
    project(pico_ism330dlc         
        VERSION 1.0.0
        LANGUAGES C CXX ASM
    )
endif()

add_subdirectory(ism330dlc_driver)
add_subdirectory(platform)

if (ISM330DLC_BUILD_COMPLEMENTARY_FILTER_EXAMPLE)
    add_subdirectory(examples/complementary_filter_demo)
endif()

if (ISM330DLC_BUILD_PAL_TEST)
    add_subdirectory(examples/hal_test)
endif()
