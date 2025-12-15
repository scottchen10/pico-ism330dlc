cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ism330dlc_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_library(ism330dlc_driver STATIC src/ism330dlc_driver.c)

target_sources(ism330dlc_driver 
    PRIVATE 
        src/ism330dlc_driver.c
)
target_include_directories(ism330dlc_driver 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

