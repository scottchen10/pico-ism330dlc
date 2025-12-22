#include "ism330dlc_pico_hal.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>


#define COMPLEMENTARY_FILTER_GYRO_WEIGHT 0.98f
#define COMPLEMENTARY_FILTER_ACCEL_WEIGHT (1 - COMPLEMENTARY_FILTER_GYRO_WEIGHT)
#define US_TO_S_CONVERSION (1.0f/1000000.0f)
#define RAD_TO_DEG_CONVERSION 57.2958f
/**
 * Represents the orientation using Trait-Bryan angles
 * Convention: Z-Y-X (Yaw, Pitch then roll)
 */
typedef struct 
{
    float roll;
    float pitch;
    float yaw;
} euler_angles_t;

void complementary_filter(
    euler_angles_t *next_orientation, 
    euler_angles_t *previous_orientation, 
    ism330dlc_gyro_t *gyro_measurements, 
    ism330dlc_accel_t *accel_measurements, 
    float delta_time_s
)    
{
    float gyro_roll = previous_orientation->roll + gyro_measurements->y * delta_time_s;
    float gyro_pitch = previous_orientation->pitch + gyro_measurements->x * delta_time_s;
    float gyro_yaw = previous_orientation->yaw + gyro_measurements->z * delta_time_s;

    float accel_roll = (float)atan(accel_measurements->z/accel_measurements->y);
    float accel_pitch = (float)atan(accel_measurements->z/accel_measurements->x);

    next_orientation->roll = COMPLEMENTARY_FILTER_ACCEL_WEIGHT * accel_roll + COMPLEMENTARY_FILTER_GYRO_WEIGHT * gyro_roll;
    next_orientation->pitch = COMPLEMENTARY_FILTER_ACCEL_WEIGHT * accel_pitch + COMPLEMENTARY_FILTER_GYRO_WEIGHT * gyro_pitch;
    next_orientation->yaw = gyro_yaw;
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

    ism330dlc_gyro_t gyro_data;
    ism330dlc_accel_t accel_data;
    float temp;

    uint8_t who_am_i;
    ism330dlc_gyro_full_scale_t gyro_fs;
    ism330dlc_accel_full_scale_t accel_fs;

    ism330dlc_read_who_am_i(&ism330dlc_sensor, &who_am_i);
    
    // The following are optional device configuration commands as they are automatically set to certain default values per the datasheet
    ism330dlc_update_accel_performance_mode(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE);
    ism330dlc_update_gyro_performance_mode(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE);
    
    ism330dlc_update_gyro_odr(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_ODR_6660_HZ);
    ism330dlc_update_accel_odr(&ism330dlc_sensor, ISM330DLC_ACCEL_GYRO_ODR_6660_HZ);
    
    ism330dlc_update_accel_full_scale(&ism330dlc_sensor, ISM330DLC_ACCEL_FS_2G);
    ism330dlc_update_gyro_full_scale(&ism330dlc_sensor, ISM330DLC_GYRO_FS_125DPS);
    
    ism330dlc_read_accel_full_scale(&ism330dlc_sensor, &accel_fs);
    ism330dlc_read_gyro_full_scale(&ism330dlc_sensor, &gyro_fs);
    
    printf("WHO_AM_I: 0x%02X \n", who_am_i);
    printf("Last Accel fs 0x%02X \n", accel_fs);
    printf("Last Gyro fs 0x%02X \n", gyro_fs);

    euler_angles_t current_orientation = {0};

    uint64_t last_poll_time = time_us_64();
    while (true) {
        uint64_t delta_time = time_us_64() - last_poll_time;

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
        last_poll_time = time_us_64();
        
        complementary_filter(
            &current_orientation,
            &current_orientation,
            &gyro_data,
            &accel_data,
            (float)delta_time * US_TO_S_CONVERSION
        );
        
        printf("----------------------------------------------\n");
        printf("Temperature        :   %.3f               Celcius \n", temp);
        printf("Accelerometer Data : [ %.3f, %.3f, %.3f ] m/s \n", accel_data.x, accel_data.y, accel_data.z);
        printf("Gyroscope Data     : [ %.3f, %.3f, %.3f ] rad/s \n", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("Fused Orientation  : [ %.3f, %.3f, %.3f ] degrees \n", 
            RAD_TO_DEG_CONVERSION * current_orientation.roll, 
            RAD_TO_DEG_CONVERSION * current_orientation.pitch, 
            RAD_TO_DEG_CONVERSION * current_orientation.yaw
        );

        sleep_us(150);
    };
};