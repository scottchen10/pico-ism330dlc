add_executable(ism330dlc_fusion_example ism330dlc_fusion_demo.c)

add_library(quaternion_lib STATIC
    external/Quaternion/Quaternion.c
)

target_include_directories(quaternion_lib 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}external/Quaternion>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ism330dlc_fusion_example
    PRIVATE
        ism330dlc_platform_pico
        quaternion_lib
)

pico_enable_stdio_usb(ism330dlc_fusion_example 1)
pico_enable_stdio_uart(ism330dlc_fusion_example 0)
pico_add_extra_outputs(ism330dlc_fusion_example)