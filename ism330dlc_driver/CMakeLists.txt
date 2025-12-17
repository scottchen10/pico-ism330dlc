add_library(ism330dlc_driver STATIC
    src/ism330dlc_driver.c
)

target_include_directories(ism330dlc_driver 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)