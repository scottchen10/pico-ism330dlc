#include "ism330dlc_pico_hal.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include "external/Quaternion/Quaternion.h"

#define COMPLEMENTARY_FILTER_ACCEL_WEIGHT 0.02f
#define RAD_TO_DEG_CONVERSION 57.2958f
#define US_TO_S_CONVERSION (1.0f/1000000.0f)
#define RAD_TO_DEG_CONVERSION 57.2958f


/**
 * Quaternion math based on the following paper
 * Valenti, R. G., Dryanovski, I., & Xiao, J. (2015). Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. Sensors, 15(8), 19302-19330. https://doi.org/10.3390/s150819302
 * See Section 5.21 for more details
 */
void calculate_accel_quaternion_estimate(ism330dlc_accel_t *accel_measurements, Quaternion *accel_estimate_out)
{
    float ax = accel_measurements->x;
    float ay = accel_measurements->y;
    float az = accel_measurements->z;

    if (accel_measurements->z >= 0.0f)
    {
        float s = sqrtf((ax + 1) * 2);
        float s_inv = 1.0f / s;
        Quaternion_set(
            s * 0.5f,
            -ay * s_inv,
            ax * s_inv,
            0,
            accel_estimate_out
        );
    }
    else
    {
        float s = sqrtf((1 - ax) * 2);
        float s_inv = 1.0f / s;
        Quaternion_set(
            -ay * s_inv,
            0.5f * s_inv,
            0.0,
            ax * s_inv,
            accel_estimate_out
        );
    }    
}

/**
 * Quaternion math based on the following paper
 * Valenti, R. G., Dryanovski, I., & Xiao, J. (2015). Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs. Sensors, 15(8), 19302-19330. https://doi.org/10.3390/s150819302
 */
void complementary_filter(
    Quaternion *next_orientation, 
    Quaternion *previous_orientation, 
    ism330dlc_gyro_t *gyro_measurements, 
    ism330dlc_accel_t *accel_measurements, 
    float delta_time_s
)    
{
    // Use small angle approximation for converting the gyro measurements to quaternion form 
    Quaternion delta_orientation;
    Quaternion gyro_estimate;
    Quaternion normalized_gyro_estimate;
    Quaternion_set(
        1.0f,
        gyro_measurements->x * delta_time_s * 0.5,
        gyro_measurements->y * delta_time_s * 0.5,
        gyro_measurements->z * delta_time_s * 0.5,
        &delta_orientation
    );

    Quaternion_multiply(previous_orientation, &delta_orientation, &gyro_estimate);
    Quaternion_normalize(&gyro_estimate, &normalized_gyro_estimate);

    // Determine orientation defined by acceleration:
    // Find the smallest arc from the global gravity frame [0,0,1]
    // Math based on fig. 25 (Valenti et al. 2015) 
    Quaternion accel_estimate;
    calculate_accel_quaternion_estimate(accel_measurements, &accel_estimate);

    Quaternion fused_estimate;
    Quaternion_slerp(
        &normalized_gyro_estimate,
        &accel_estimate,
        COMPLEMENTARY_FILTER_ACCEL_WEIGHT,
        &fused_estimate
    );
    
    Quaternion_normalize(&fused_estimate, &next_orientation);
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

    Quaternion orientation_a;
    Quaternion orientation_b;

    Quaternion *current_orientation_ptr = &orientation_a;
    Quaternion *next_orientation_ptr = &orientation_b;
    Quaternion *temp_orientation_ptr = &orientation_a;

    double euler_angles[3];

    // Determine the initial state using accel data
    ism330dlc_read_raw_accel_data(&ism330dlc_sensor, &raw_accel_data);
    ism330dlc_convert_raw_accel_xyz_to_mps2(
        ism330dlc_sensor.last_accel_fs,
        &raw_accel_data,
        &accel_data
    );
    calculate_accel_quaternion_estimate(&accel_data, current_orientation_ptr);

    // Update the orientation over time and output the estimated state
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
            next_orientation_ptr,
            current_orientation_ptr,
            &gyro_data,
            &accel_data,
            (float)delta_time * US_TO_S_CONVERSION
        );

        Quaternion_toEulerZYX(next_orientation_ptr, euler_angles);
        
        printf("----------------------------------------------\n");
        printf("Temperature        :   %.3f               Celcius \n", temp);
        printf("Accelerometer Data : [ %.3f, %.3f, %.3f ] m/s \n", accel_data.x, accel_data.y, accel_data.z);
        printf("Gyroscope Data     : [ %.3f, %.3f, %.3f ] rad/s \n", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("Fused Orientation  : [ %.3f, %.3f, %.3f ] degrees \n", 
            RAD_TO_DEG_CONVERSION * euler_angles[0], 
            RAD_TO_DEG_CONVERSION * euler_angles[1], 
            RAD_TO_DEG_CONVERSION * euler_angles[2]
        );

        temp_orientation_ptr = next_orientation_ptr;
        next_orientation_ptr = current_orientation_ptr;
        current_orientation_ptr = temp_orientation_ptr;
        sleep_us(150);
    };
};