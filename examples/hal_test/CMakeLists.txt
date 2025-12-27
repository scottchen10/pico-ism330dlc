add_executable(ism330dlc_pico_hal_test ism330dlc_pico_hal_test.c)

target_link_libraries(ism330dlc_pico_hal_test
    PUBLIC 
        ism330dlc_platform_pico
)

pico_enable_stdio_usb(ism330dlc_pico_hal_test 1)
pico_enable_stdio_uart(ism330dlc_pico_hal_test 0)
pico_add_extra_outputs(ism330dlc_pico_hal_test)