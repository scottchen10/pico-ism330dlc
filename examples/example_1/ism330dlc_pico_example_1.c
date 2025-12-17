#include "ism330dlc_pico_hal.h"

void complementary_filter() 
{

};

void main() 
{
    const ism330dlc_pico_i2c_config i2c_config = {
        .port        = i2c_default,
        .i2c_address = ISM330DLC_I2C_ADDR0,
        .baudrate    = 115200,
        .cs_pin      = 1,
        .scl_pin     = 2,
        .sda_pin     = 3
    };

    ism330dlc_pico_i2c_bus_init(i2c_config);
    ism330dlc_pico_i2c_pins_init(i2c_config);

    ism330dlc_t ism330dlc_sensor;

    ism330dlc_init(
        &ism330dlc_sensor,
        &i2c_config,
        ISM330DLC_BUS_I2C
    );
};