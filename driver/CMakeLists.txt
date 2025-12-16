add_library(ism330dlc_driver STATIC)

target_sources(ism330dlc_driver 
    PRIVATE 
        src/ism330dlc_driver.c
)
target_include_directories(ism330dlc_driver 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)