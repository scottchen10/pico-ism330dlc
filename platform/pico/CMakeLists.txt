add_library(ism330dlc_platform_pico STATIC)

target_sources(ism330dlc_platform_pico
    PRIVATE 
        ism330dlc_pico_hal.c
)

target_include_directories(ism330dlc_platform_pico 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ism330dlc_platform_pico
    PUBLIC 
        ism330dlc_driver
        pico_stdlib 
        hardware_i2c
        hardware_spi
        hardware_gpio
)