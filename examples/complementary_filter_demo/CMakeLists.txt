add_executable(ism330dlc_fusion_example ism330dlc_fusion_demo.c)

add_subdirectory(external/Quaternion)

target_link_libraries(ism330dlc_fusion_example
    PRIVATE
        ism330dlc_platform_pico
        quaternion
)

pico_enable_stdio_usb(ism330dlc_fusion_example 1)
pico_enable_stdio_uart(ism330dlc_fusion_example 0)
pico_add_extra_outputs(ism330dlc_fusion_example)