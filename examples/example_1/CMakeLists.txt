add_executable(ism330dlc_pico_example_1 ism330dlc_pico_example_1.c)

target_link_libraries(ism330dlc_pico_example_1
    PUBLIC 
        ism330dlc_platform_pico
)