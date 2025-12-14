cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico-ism330dlc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_library(pico-ism330dlc STATIC)

target_link_libraries(pico-ism330dlc
    pico_stdlib
    hardware_i2c
    hardware_spi
)

target_sources(pico-ism330dlc 
    PRIVATE 
        src/ism330dlc_driver.c
        platform/pico/ism330dlc_pico_hal.c
)

target_include_directories(pico-ism330dlc 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/platform
)

