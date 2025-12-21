add_executable(ism330dlc_orientation_example ism330dlc_fusion_demo.c)

target_link_libraries(ism330dlc_orientation_example
    PUBLIC 
        ism330dlc_platform_pico
)

pico_enable_stdio_usb(ism330dlc_orientation_example 1)
pico_enable_stdio_uart(ism330dlc_orientation_example 0)
pico_add_extra_outputs(ism330dlc_orientation_example)