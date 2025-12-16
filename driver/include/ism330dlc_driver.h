#ifndef ISM330DLC_DRIVER_H
#define ISM330DLC_DRIVER_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    ISM330DLC_SUCCESS = 0x01,
    ISM330DLC_ERROR = 0x00
} ism330dlc_status_t;

typedef enum
{
    ISM330DLC_BUS_SPI3,
    ISM330DLC_BUS_SPI4,
    ISM330DLC_BUS_I2C
} ism330dlc_bus_type_t;

typedef enum
{
    ISM330DLC_I2C_ADDR0 = 0x6A,
    ISM330DLC_I2C_ADDR1 = 0x6B
} ism330dlc_i2c_address_t;

typedef struct
{
    void* device_context;
    ism330dlc_gyro_full_scale_t last_gyro_fs;
    ism330dlc_accel_full_scale_t last_accel_fs;
    ism330dlc_status_t (*read_registers)(void *device_context, uint8_t address, uint8_t *buffer, size_t length);
    ism330dlc_status_t (*write_registers)(void *device_context, uint8_t address, uint8_t *data, size_t length);
} ism330dlc_t;

void ism330dlc_init(
    ism330dlc_t *instance, 
    void* device_context, 
    ism330dlc_status_t (*read_registers)(void *device_context, uint8_t address, uint8_t *buffer, size_t length),
    ism330dlc_status_t (*write_registers)(void *device_context, uint8_t address, uint8_t *data, size_t length)
);

/**
 * @brief Reads the WHO_AM_I register
 * 
 * The WHO_AM_I register should return 0x6A or 0x6B for a valid ISM330DLC device
 * 
 * @param result Pointer to where the 1 byte register value will be stored
 * @return Success or error status code
 */
ism330dlc_status_t ism330dlc_read_who_am_i(ism330dlc_t* device, uint8_t* result) ;

typedef union {
    uint16_t u16;
    uint8_t u8[2];
} ism330dlc_reg16_t;

typedef union {
    struct {
        uint16_t x;
        uint16_t y;
        uint16_t z;  
    } axis;
    uint8_t values[6];
} ism330dlc_raw_xyz_t;

/**
 * @brief Reads the raw accelerometer data.
 * 
 * @param device Pointer to the ISM330DLC device instance.
 * @param result Pointer to where the raw xyz accelerometer data will be stored.
 * @return ISM330DLC status code
 */
ism330dlc_status_t ism330dlc_read_raw_accel_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result);

/**
 * @brief Reads the raw gyroscope data.
 * 
 * @param device Pointer to the ISM330DLC device instance.
 * @param result Pointer to where the raw xyz gyroscope data will be stored.
 * @return ISM330DLC status code
 */
ism330dlc_status_t ism330dlc_read_raw_gyro_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result);

/**
 * @brief Reads the raw temperature data.
 * 
 * @param device Pointer to the ISM330DLC device instance.
 * @param result Pointer to where the raw temperature data will be stored.
 * @return status [ISM330DLC_SUCCESS, ISM330DLC_ERROR]
 */
ism330dlc_status_t ism330dlc_read_raw_temperature_data(ism330dlc_t* device, ism330dlc_reg16_t *result);

typedef enum 
{
    ISM330DLC_ACCEL_GYRO_ODR_POWER_DOWN = 0x00,
    ISM330DLC_ACCEL_GYRO_ODR_1_6_HZ     = 0xB0,
    ISM330DLC_ACCEL_GYRO_ODR_12_5_HZ    = 0x10,
    ISM330DLC_ACCEL_GYRO_ODR_26_HZ      = 0x20,
    ISM330DLC_ACCEL_GYRO_ODR_52_HZ      = 0x30,
    ISM330DLC_ACCEL_GYRO_ODR_104_HZ     = 0x40,
    ISM330DLC_ACCEL_GYRO_ODR_208_HZ     = 0x50,
    ISM330DLC_ACCEL_GYRO_ODR_416_HZ     = 0x50,
    ISM330DLC_ACCEL_GYRO_ODR_833_HZ     = 0x50,
    ISM330DLC_ACCEL_GYRO_ODR_1660_HZ    = 0x50,
    ISM330DLC_ACCEL_GYRO_ODR_3330_HZ    = 0x50,
    ISM330DLC_ACCEL_GYRO_ODR_6660_HZ    = 0x50,
} ism330dlc_accel_gyro_odr_t;

typedef enum 
{
    ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE  = 0x00,
    ISM330DLC_ACCEL_GYRO_LOW_PERFORMANCE   = 0x10,

} ism330dlc_accel_gyro_performance_mode_t;

/**
 * @brief Updates the accelerometer performance mode.
 *
 * Configures the accelerometer for high-performance or low-power operation.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param mode   Accelerometer performance mode.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_update_accel_performance_mode(ism330dlc_t* device, ism330dlc_accel_gyro_performance_mode_t mode);

/**
 * @brief Updates the accelerometer output data rate.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param odr    Target sampling rate.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_update_accel_odr(ism330dlc_t* device, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Updates the gyroscope performance mode.
 *
 * Configures the gyroscope for high-performance or low-power operation.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param mode   Gyroscope performance mode.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_update_gyro_performance_mode(ism330dlc_t* device, ism330dlc_accel_gyro_performance_mode_t mode);

/**
 * @brief Updates the gyroscope output data rate.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param odr    Target sampling rate.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_update_gyro_odr(ism330dlc_t* device, ism330dlc_accel_gyro_odr_t odr);

typedef enum 
{
    ISM330DLC_ACCEL_FULL_SCALE_2G  = 0x00,
    ISM330DLC_ACCEL_FULL_SCALE_16G = 0x04,
    ISM330DLC_ACCEL_FULL_SCALE_4G  = 0x08,
    ISM330DLC_ACCEL_FULL_SCALE_8G  = 0x0C,
} ism330dlc_accel_full_scale_t;

/**
 * @brief Updates the accelerometer measurement scale.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Target measurement scale.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_update_accel_full_scale(ism330dlc_t* device, ism330dlc_accel_full_scale_t scale);

/**
 * @brief Reads the accelerometer measurement scale and caches the result
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Pointer to where the measurement scale will be stored
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_read_accel_full_scale(ism330dlc_t* device, ism330dlc_accel_full_scale_t* scale);

typedef enum 
{
    ISM330DLC_GYRO_FULL_SCALE_125DPS   = 0x02,
    ISM330DLC_GYRO_FULL_SCALE_250DPS   = 0x00,
    ISM330DLC_GYRO_FULL_SCALE_500DPS   = 0x04,
    ISM330DLC_GYRO_FULL_SCALE_1000DPS  = 0x08,
    ISM330DLC_GYRO_FULL_SCALE_2000DPS  = 0x0C,
} ism330dlc_gyro_full_scale_t;

/**
 * @brief Updates the gyroscope measurement scale.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Target measurement scale.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_update_gyro_full_scale(ism330dlc_t* device, ism330dlc_gyro_full_scale_t scale);

/**
 * @brief Reads the gyroscope measurement scale and caches the result
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Pointer to where the measurement scale will be stored
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_read_gyro_full_scale(ism330dlc_t* device, ism330dlc_gyro_full_scale_t* scale);


#endif
