#include "ism330dlc_pico_pal.h"
#include "ism330dlc/ism330dlc_regs.h"
#include "pico/stdlib.h"
#include "stdio.h"

#define SPI4_TEST

int main(void)
{
    stdio_init_all();
    sleep_ms(3000);

#ifdef I2C_TEST
    ism330dlc_pico_i2c_config handle = {
        .port        = i2c_default,
        .i2c_address = ISM330DLC_I2C_ADDR0,
        .baudrate    = 400000,
        .sda_pin     = 0,
        .scl_pin     = 1,
        .cs_pin      = 2
    };

    ism330dlc_pico_i2c_pins_init(handle);
    ism330dlc_pico_i2c_bus_init(handle);

    ism330dlc_t ism330dlc_sensor;
    ism330dlc_init(
        &ism330dlc_sensor,
        &handle,
        ISM330DLC_BUS_I2C
    );
#elifdef SPI4_TEST 
    ism330dlc_pico_spi4_config handle = {
        .port        = spi_default,
        .baudrate    = 1000000,
        .sdo_pin     = 0,
        .sdi_pin     = 3,
        .scl_pin     = 2,
        .cs_pin      = 1
    };

    ism330dlc_pico_spi4_pins_init(handle);
    ism330dlc_pico_spi4_bus_init(handle);

    ism330dlc_t ism330dlc_sensor;
    ism330dlc_init(
        &ism330dlc_sensor,
        &handle,
        ISM330DLC_BUS_SPI4
    );

#endif


    // Verify reading registers work

    uint8_t who_am_i = 0x99;
    ism330dlc_status_t resp = ism330dlc_read_who_am_i(&ism330dlc_sensor, &who_am_i);
    if (resp != ISM330DLC_SUCCESS)
    {
        printf("Failed to read WHO_AM_I \n");
    }
    else if (who_am_i == 0x99)
    {
        printf("Silently failed to read WHO_AM_I \n", who_am_i);       
    }
    else if (who_am_i != 0x6A && who_am_i != 0x6B)
    {
        printf("Invalid data received from WHO_AM_I: 0x%02X \n", who_am_i);
    }
    else 
    {
        printf("Successfully read WHO_AM_I: 0x%02X \n", who_am_i);
    }

    // Verify writing to registers work

    ism330dlc_accel_full_scale_t stored_scale = 0x99;
    resp = ism330dlc_set_accel_full_scale(&ism330dlc_sensor, ISM330DLC_ACCEL_FS_16G);
    if (resp != ISM330DLC_SUCCESS)
        printf("Failed to update accelerometer full scale \n");

    resp =  ism330dlc_read_accel_full_scale(&ism330dlc_sensor, &stored_scale);
    if (resp != ISM330DLC_SUCCESS)
        printf("Failed to read accelerometer full scale \n");

    if (stored_scale != ISM330DLC_ACCEL_FS_16G)
    {
        printf("Failed to update the stored scale: 0x%02X \n", (uint8_t)stored_scale);
    } 
    else 
    {
        printf("Successfully updated the full scale \n");
    }


    uint8_t target_reg = 0xFF;
    uint8_t result_reg = 0x11;

    resp = ism330dlc_sensor.write_registers(
        &ism330dlc_sensor.device_context, 
        ISM330DLC_ADDR_CTRL1_XL,
        &target_reg,
        1
    );
    if (resp != ISM330DLC_SUCCESS)
        printf("Failed to write to CTRL1_XL \n");
    resp = ism330dlc_sensor.read_registers(
        &ism330dlc_sensor.device_context, 
        ISM330DLC_ADDR_CTRL1_XL,
        &result_reg,
        1
    );   
    if (resp != ISM330DLC_SUCCESS)
        printf("Failed to read from CTRL1_XL \n");

    if (result_reg != target_reg)
    {
        printf("Failed to update CTRL1_XL: 0x%02X \n", (uint8_t)result_reg);
    } 
    else 
    {
        printf("Successfully wrote to CTRL1_XL: 0x%02X \n", (uint8_t)result_reg);
    }

    return 0;
};