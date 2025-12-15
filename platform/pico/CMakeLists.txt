#set(PICO_BOARD pico)
#set(PICO_BOARD pico_w)
set(PICO_BOARD pico2)

# If pico2 is selected you have to select PICO_PLATFORM
# set(PICO_PLATFORM rp2350-arm-s)
set(PICO_PLATFORM rp2350-riscv)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init() 

add_subdirectory(${CMAKE_SOURCE_DIR}/../../..)

add_library(ism330dlc_platform_pico STATIC ism330dlc_pico_hal.c)

target_link_libraries(ism330dlc_platform_pico PRIVATE 
    pico_stdlib 
    hardware_i2c
    ism330dlc_driver
)

target_include_directories(ism330dlc_platform_pico
    PRIVATE 
        ${CMAKE_SOURCE_DIR}/../../../include 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}
)