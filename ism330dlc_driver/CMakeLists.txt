add_library(ism330dlc_driver INTERFACE)

target_sources(ism330dlc_driver 
    INTERFACE
        src/ism330dlc_driver.c
)

target_include_directories(ism330dlc_driver 
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)