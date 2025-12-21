add_executable(ism330dlc_pico_example_1 ism330dlc_pico_example_1.c)

target_link_libraries(ism330dlc_pico_example_1
    PUBLIC 
        ism330dlc_platform_pico
)

pico_enable_stdio_usb(ism330dlc_pico_example_1 1)
pico_enable_stdio_uart(ism330dlc_pico_example_1 0)
pico_add_extra_outputs(ism330dlc_pico_example_1)