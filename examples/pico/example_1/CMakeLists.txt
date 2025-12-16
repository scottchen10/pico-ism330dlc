add_executable(ism330dlc_pico_example_1)

target_sources(ism330dlc_pico_example_1
    PRIVATE 
        ism330dlc_pico_example_1.c
)

target_link_libraries(ism330dlc_pico_example_1
    PUBLIC 
        ism330dlc_platform_pico
)

target_include_directories(ism330dlc_pico_example_1 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)
