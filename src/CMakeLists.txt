add_library(ism330dlc 
    INTERFACE
)

target_sources(ism330dlc
    INTERFACE
        ism330dlc_driver.c ism330dlc_driver.h
)

target_include_directories(ism330dlc
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(ism330dlc 
    INTERFACE
        hardware_gpio
        hardware_i2c
)