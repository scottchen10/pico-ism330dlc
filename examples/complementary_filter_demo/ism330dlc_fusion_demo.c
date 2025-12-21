#include "ism330dlc_pico_hal.h"
#include "pico/stdlib.h"
#include "stdio.h"

void complementary_filter() 
{

};

void main() 
{
    stdio_init_all();
    sleep_ms(2500);

    ism330dlc_pico_i2c_config i2c_config = {
        .port        = i2c_default,
        .i2c_address = ISM330DLC_I2C_ADDR0,
        .baudrate    = 400000,
        .cs_pin      = 2,
        .scl_pin     = 1,
        .sda_pin     = 0
    };

    ism330dlc_pico_i2c_pins_init(i2c_config);
    ism330dlc_pico_i2c_bus_init(i2c_config);

    ism330dlc_t ism330dlc_sensor;

    ism330dlc_init(
        &ism330dlc_sensor,
        &i2c_config,
        ISM330DLC_BUS_I2C
    );

    ism330dlc_raw_xyz_t raw_accel_data;
    ism330dlc_raw_xyz_t raw_gyro_data;
    ism330dlc_reg16_t raw_temp_data;

    ism330dlc_gyro_t gyro_data = {
        .x = 11.111f,
        .y = 11.111f,
        .z = 11.111f,
    };
    ism330dlc_accel_t accel_data= {
        .x = 11.111f,
        .y = 11.111f,
        .z = 11.111f,
    };
    float temp = 11.111f;

    uint8_t who_am_i = 0x11;
    ism330dlc_gyro_full_scale_t gyro_fs = 0x11;
    ism330dlc_accel_full_scale_t accel_fs = 0x11;

    printf("INITALIZING \n");
    ism330dlc_read_who_am_i(&ism330dlc_sensor, &who_am_i);
    printf("WHO_AM_I: 0x%02X \n", who_am_i);

    // The following are optional device configuration commands as they are automatically set to certain default values per the datasheet
    ism330dlc_update_accel_performance_mode(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE);
    ism330dlc_update_gyro_performance_mode(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE);

    ism330dlc_update_gyro_odr(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_ODR_3330_HZ);
    ism330dlc_update_accel_odr(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_ODR_3330_HZ);

    ism330dlc_update_accel_full_scale(&ism330dlc_sensor, ISM330DLC_ACCEL_FS_4G);
    ism330dlc_update_gyro_full_scale(&ism330dlc_sensor, ISM330DLC_GYRO_FS_500DPS);

    ism330dlc_read_accel_full_scale(&ism330dlc_sensor, &accel_fs);
    ism330dlc_read_gyro_full_scale(&ism330dlc_sensor, &gyro_fs);

    printf("Last Accel fs 0x%02X \n", accel_fs);
    printf("Last Gyro fs 0x%02X \n", gyro_fs);


    while (true) {
        ism330dlc_read_raw_accel_data(&ism330dlc_sensor, &raw_accel_data);
        ism330dlc_read_raw_gyro_data(&ism330dlc_sensor, &raw_gyro_data);
        ism330dlc_read_raw_temperature_data(&ism330dlc_sensor, &raw_temp_data);

        ism330dlc_convert_raw_accel_xyz_to_mps2(
            ism330dlc_sensor.last_accel_fs,
            &raw_accel_data,
            &accel_data
        );

        ism330dlc_convert_raw_gyro_xyz_to_rps(
            ism330dlc_sensor.last_gyro_fs,
            &raw_gyro_data,
            &gyro_data
        );

        temp = ism330dlc_convert_raw_temp_to_celcius(raw_temp_data.u16);

        printf("Gyroscope Data     : [ %.3f, %.3f, %.3f ] rad/s \n", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("Accelerometer Data : [ %.3f, %.3f, %.3f ] m/s \n", accel_data.x, accel_data.y, accel_data.z);
        printf("Temperature        :   %.3f           Celcius \n", temp);

        sleep_ms(500);
    };
};