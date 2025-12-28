add_library(ism330dlc_platform_pico STATIC
    ism330dlc_pico_pal.c
)

target_link_libraries(ism330dlc_platform_pico
    PUBLIC 
        ism330dlc_driver
        pico_stdlib 
        hardware_i2c
        hardware_spi
        hardware_gpio
)

target_include_directories(ism330dlc_platform_pico 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)
