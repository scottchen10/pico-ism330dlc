#ifndef ISM330DLC_DRIVER_H
#define ISM330DLC_DRIVER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ism330dlc_driver/ism330dlc_regs.h"

typedef enum
{
    ISM330DLC_SUCCESS = 0x01,
    ISM330DLC_ERROR   = 0x00
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

typedef enum
{
    ISM330DLC_ACCEL_FS_2G  = 0x00,
    ISM330DLC_ACCEL_FS_16G = 0x04,
    ISM330DLC_ACCEL_FS_4G  = 0x08,
    ISM330DLC_ACCEL_FS_8G  = 0x0C,
} ism330dlc_accel_full_scale_t;

typedef enum
{
    ISM330DLC_GYRO_FS_125DPS  = 0x02,
    ISM330DLC_GYRO_FS_250DPS  = 0x00,
    ISM330DLC_GYRO_FS_500DPS  = 0x04,
    ISM330DLC_GYRO_FS_1000DPS = 0x08,
    ISM330DLC_GYRO_FS_2000DPS = 0x0C,
} ism330dlc_gyro_full_scale_t;

typedef struct
{
    void *device_context;
    ism330dlc_gyro_full_scale_t last_gyro_fs;
    ism330dlc_accel_full_scale_t last_accel_fs;
    ism330dlc_status_t (*read_registers)(void *device_context, uint8_t address, uint8_t *buffer, size_t length);
    ism330dlc_status_t (*write_registers)(void *device_context, uint8_t address, uint8_t *data, size_t length);
} ism330dlc_t;

typedef enum
{
    ISM330DLC_INIT_SUCCESS = 0x00,
    ISM330DLC_INIT_FAIL_INVALID_BUS,
    ISM330DLC_INIT_FAIL_SPI3_DISABLED,
} ism330dlc_init_status;

ism330dlc_init_status ism330dlc_init(
    ism330dlc_t *instance,
    void *device_context,
    ism330dlc_bus_type_t bus_type
);

extern ism330dlc_status_t ism330dlc_i2c_read_registers(void *handle, uint8_t address, uint8_t *buffer, size_t length);
extern ism330dlc_status_t ism330dlc_i2c_write_registers(void *handle, uint8_t address, uint8_t *data, size_t length);
extern ism330dlc_status_t ism330dlc_spi3_read_registers(void *handle, uint8_t address, uint8_t *buffer, size_t length);
extern ism330dlc_status_t ism330dlc_spi3_write_registers(void *handle, uint8_t address, uint8_t *data, size_t length);
extern ism330dlc_status_t ism330dlc_spi4_read_registers(void *handle, uint8_t address, uint8_t *buffer, size_t length);
extern ism330dlc_status_t ism330dlc_spi4_write_registers(void *handle, uint8_t address, uint8_t *data, size_t length);

/**
 * @brief Reads the WHO_AM_I register
 *
 * The WHO_AM_I register should return 0x6A or 0x6B for a valid ISM330DLC device
 *
 * @param result Pointer to where the 1 byte register value will be stored
 * @return Success or error status code
 */
ism330dlc_status_t ism330dlc_read_who_am_i(ism330dlc_t *device, uint8_t *result);

typedef union
{
    int16_t u16;
    uint8_t bytes[2];
} ism330dlc_reg16_t;

typedef union
{
    int16_t axes[3];
    uint8_t bytes[6];
} ism330dlc_raw_xyz_t;

/**
 * @brief Reads the raw accelerometer data.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param result Pointer to where the raw xyz accelerometer data will be stored.
 * @return ISM330DLC status code
 */
ism330dlc_status_t ism330dlc_read_raw_accel_data(ism330dlc_t *device, ism330dlc_raw_xyz_t *result);

/**
 * @brief Reads the raw gyroscope data.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param result Pointer to where the raw xyz gyroscope data will be stored.
 * @return ISM330DLC status code
 */
ism330dlc_status_t ism330dlc_read_raw_gyro_data(ism330dlc_t *device, ism330dlc_raw_xyz_t *result);

/**
 * @brief Reads the raw temperature data.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param result Pointer to where the raw temperature data will be stored.
 * @return status [ISM330DLC_SUCCESS, ISM330DLC_ERROR]
 */
ism330dlc_status_t ism330dlc_read_raw_temperature_data(ism330dlc_t *device, ism330dlc_reg16_t *result);

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
    ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE = 0x00,
    ISM330DLC_ACCEL_GYRO_LOW_PERFORMANCE  = 0x10,

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
ism330dlc_status_t ism330dlc_set_accel_performance_mode(ism330dlc_t *device, ism330dlc_accel_gyro_performance_mode_t mode);

/**
 * @brief Updates the accelerometer output data rate.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param odr    Target sampling rate.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_accel_odr(ism330dlc_t *device, ism330dlc_accel_gyro_odr_t odr);

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
ism330dlc_status_t ism330dlc_set_gyro_performance_mode(ism330dlc_t *device, ism330dlc_accel_gyro_performance_mode_t mode);

/**
 * @brief Updates the gyroscope output data rate.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param odr    Target sampling rate.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_gyro_odr(ism330dlc_t *device, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Updates the accelerometer measurement scale.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Target measurement scale.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_accel_full_scale(ism330dlc_t *device, ism330dlc_accel_full_scale_t scale);

/**
 * @brief Reads the accelerometer measurement scale and caches the result
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Pointer to where the measurement scale will be stored
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_read_accel_full_scale(ism330dlc_t *device, ism330dlc_accel_full_scale_t *scale);

/**
 * @brief Updates the gyroscope measurement scale.
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Target measurement scale.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_gyro_full_scale(ism330dlc_t *device, ism330dlc_gyro_full_scale_t scale);

/**
 * @brief Reads the gyroscope measurement scale and caches the result
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param scale  Pointer to where the measurement scale will be stored
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_read_gyro_full_scale(ism330dlc_t *device, ism330dlc_gyro_full_scale_t *scale);

typedef struct 
{
    float x, y, z;
} ism330dlc_accel_t;

/**
 * @brief Converts the raw accelerometer data into units of g based on the provided scale
 *
 * @param scale The measurement scale
 * @param raw_values The raw measurement data
 * @param accel_g Pointer to where the converted values in g will be stored
 *
 */
void ism330dlc_convert_raw_accel_xyz_to_g(ism330dlc_accel_full_scale_t scale, const ism330dlc_raw_xyz_t *raw_values, ism330dlc_accel_t *accel_g);

/**
 * @brief Converts the raw accelerometer data into meters per second squared based on the provided scale
 *
 * @param scale The measurement scale
 * @param raw_values Pointer to the raw measurement data
 * @param accel_mps Pointer to where the converted values in m/s/s in will be stored
 */
void ism330dlc_convert_raw_accel_xyz_to_mps2(ism330dlc_accel_full_scale_t scale, const ism330dlc_raw_xyz_t *raw_values, ism330dlc_accel_t *accel_mps);

typedef struct 
{
    float x, y, z;
} ism330dlc_gyro_t;

/**
 * @brief Converts the raw gyroscope data into degrees per second based on the provided scale
 *
 * @param scale The measurement scale
 * @param raw_values Pointer to the raw measurement data
 * @param gyro_dps Pointer to where the converted values in dps will be stored
 */
void ism330dlc_convert_raw_gyro_xyz_to_dps(ism330dlc_gyro_full_scale_t scale, const ism330dlc_raw_xyz_t *raw_values, ism330dlc_gyro_t *gyro_dps);

/**
 * @brief Converts the raw gyroscope data into radians per second based on the provided scale
 *
 * @param scale The measurement scale
 * @param raw_values Pointer to the raw measurement data
 * @param gyro_rps Pointer to where the converted values in rad/s will be stored
 */
void ism330dlc_convert_raw_gyro_xyz_to_rps(ism330dlc_gyro_full_scale_t scale, const ism330dlc_raw_xyz_t *raw_values, ism330dlc_gyro_t *gyro_rps);

/**
 * @brief Converts the raw temperature data to degrees celcius
 *
 * @param temp The raw temperature data
 * @return The temperature in degrees celcius
 */
float ism330dlc_convert_raw_temp_to_celcius(int16_t temp);


typedef enum 
{
    ISM330DLC_INTERRUPT_ACTIVE_HIGH = 0x00,
    ISM330DLC_INTERRUPT_ACTIVE_LOW  = 0x20,
} ism330dlc_interrupt_active_mode_t;

ism330dlc_status_t ism330dlc_set_interrupt_active_mode(ism330dlc_t *device, ism330dlc_interrupt_active_mode_t mode);

typedef enum 
{
    ISM330DLC_INTERRUPT_PUSH_PULL  = 0x00,
    ISM330DLC_INTERRUPT_OPEN_DRAIN = 0x10,
} ism330dlc_interrupt_output_mode_t;

ism330dlc_status_t ism330dlc_set_interrupt_output_mode(ism330dlc_t *device, ism330dlc_interrupt_output_mode_t mode);

ism330dlc_status_t ism330dlc_set_event_interrupts_enable(bool are_event_driven_interrupts_enabled);

typedef enum 
{
    ISM330DLC_ACTIVITY_EVENT_DISABLED        = 0x00,
    ISM330DLC_ACTIVITY_EVENT_GYRO_UNCHANGED  = 0x20,
    ISM330DLC_ACTIVITY_EVENT_GYRO_SLEEP      = 0x40,
    ISM330DLC_ACTIVITY_EVENT_GYRO_POWER_DOWN = 0x60,

} ism330dlc_inactivity_event_mode_t;

ism330dlc_status_t ism330dlc_set_inactivity_event_mode(ism330dlc_inactivity_event_mode_t mode);

typedef enum
{
    ISM330DLC_EVENT_TAP_X_EN = 0x08,
    ISM330DLC_EVENT_TAP_Y_EN = 0x04,
    ISM330DLC_EVENT_TAP_Z_EN = 0x02,
} ism330dlc_tap_event_axis_t;

ism330dlc_status_t ism330dlc_set_tap_event_axis_enable(ism330dlc_tap_event_axis_t axis, bool enabled);

typedef enum
{
    ISM330DLC_INTERRUPT_UNLATCHED = 0x00,
    ISM330DLC_INTERRUPT_LATCHED
} ism330dlc_interrupt_latched_mode_t;

ism330dlc_status_t ism330dlc_set_interrupt_latched_mode(ism330dlc_interrupt_latched_mode_t mode);

typedef enum
{
    ISM330DLC_MD_EVENT_INACT_STATE = ISM330DLC_MASK_MD_INACT_STATE,
    ISM330DLC_MD_EVENT_SINGLE_TAP  = ISM330DLC_MASK_MD_SINGLE_TAP,
    ISM330DLC_MD_EVENT_WU          = ISM330DLC_MASK_MD_WU,
    ISM330DLC_MD_EVENT_FF          = ISM330DLC_MASK_MD_FF,
    ISM330DLC_MD_EVENT_DOUBLE_TAP  = ISM330DLC_MASK_MD_DOUBLE_TAP,
    ISM330DLC_MD_EVENT_6D          = ISM330DLC_MASK_MD_6D,
    ISM330DLC_MD_EVENT_TILT        = ISM330DLC_MASK_MD_TILT,
} ism330dlc_md_event_t;

typedef enum
{
    ISM330DLC_INTERRUPT_PIN_1 = 0,
    ISM330DLC_INTERRUPT_PIN_2,
} ism330dlc_interrupt_pin_t;

ism330dlc_status_t ism330dlc_set_event_interrupt_route(ism330dlc_interrupt_pin_t pin, ism330dlc_md_event_t event, bool enabled);

typedef enum
{
    ISM330DLC_6D_THS_80_DEG = 0x00,
    ISM330DLC_6D_THS_70_DEG = 0x20,
    ISM330DLC_6D_THS_60_DEG = 0x40,
    ISM330DLC_6D_THS_50_DEG = 0x60,
} ism330dlc_6d_threshold_t;

typedef enum
{
    ISM330DLC_INACT_DISABLED      = 0x00,
    ISM330DLC_INACT_XL_12HZ       = 0x20,
    ISM330DLC_INACT_XL_12HZ_G_SLP = 0x40,
    ISM330DLC_INACT_XL_12HZ_G_PD  = 0x60,
} ism330dlc_inact_mode_t;

ism330dlc_status_t ism330dlc_set_wakeup_threshold(uint8_t threshold);
ism330dlc_status_t ism330dlc_set_wakeup_duration(uint8_t duration);

ism330dlc_status_t ism330dlc_set_freefall_threshold(uint8_t threshold);
ism330dlc_status_t ism330dlc_set_freefall_duration(uint8_t duration);

ism330dlc_status_t ism330dlc_set_tap_threshold(uint8_t threshold);
ism330dlc_status_t ism330dlc_set_tap_timing(uint8_t shock, uint8_t quiet, uint8_t duration);
ism330dlc_status_t ism330dlc_set_double_tap_enable(bool enabled);

ism330dlc_status_t ism330dlc_set_6d_threshold(ism330dlc_6d_threshold_t threshold);
ism330dlc_status_t ism330dlc_set_4d_orientation_enable(bool enabled);

ism330dlc_status_t ism330dlc_set_inactivity_mode(ism330dlc_inact_mode_t mode);
ism330dlc_status_t ism330dlc_set_sleep_duration(uint8_t duration);

#endif
