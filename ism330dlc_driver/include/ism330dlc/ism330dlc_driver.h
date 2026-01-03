#ifndef ISM330DLC_DRIVER_H
#define ISM330DLC_DRIVER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "ism330dlc/ism330dlc_regs.h"

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
    ISM330DLC_INIT_SUCCESS = 0,
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
    ISM330DLC_ACCEL_GYRO_ODR_416_HZ     = 0x60,
    ISM330DLC_ACCEL_GYRO_ODR_833_HZ     = 0x70,
    ISM330DLC_ACCEL_GYRO_ODR_1660_HZ    = 0x80,
    ISM330DLC_ACCEL_GYRO_ODR_3330_HZ    = 0x90,
    ISM330DLC_ACCEL_GYRO_ODR_6660_HZ    = 0xA0,
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
 * 
 * @return The temperature in degrees celcius
 */
float ism330dlc_convert_raw_temp_to_celcius(int16_t temp);

typedef enum 
{
    ISM330DLC_INTERRUPT_ACTIVE_HIGH = 0x00,
    ISM330DLC_INTERRUPT_ACTIVE_LOW  = 0x20,
} ism330dlc_interrupt_active_mode_t;

/**
 * @brief Sets the interrupt active mode (polarity) for the interrupt pins
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param mode   The interrupt active mode (Active High or Active Low).
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_interrupt_active_mode(ism330dlc_t *device, ism330dlc_interrupt_active_mode_t mode);

typedef enum 
{
    ISM330DLC_INTERRUPT_PUSH_PULL  = 0x00,
    ISM330DLC_INTERRUPT_OPEN_DRAIN = 0x10,
} ism330dlc_interrupt_output_mode_t;


/**
 * @brief Sets the interrupt pin output modes (Push/Pull or Open Drain modes)
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param mode   The output mode (Push-Pull or Open-Drain).
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_interrupt_output_mode(ism330dlc_t *device, ism330dlc_interrupt_output_mode_t mode);

/**
 * @brief Enables or disables event-driven interrupts globally
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param are_event_driven_interrupts_enabled   
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_event_interrupts_enable(ism330dlc_t *device, bool are_event_driven_interrupts_enabled);

typedef enum 
{
    ISM330DLC_INACTIVITY_EVENT_DISABLED        = 0x00,
    ISM330DLC_INACTIVITY_EVENT_GYRO_UNCHANGED  = 0x20,
    ISM330DLC_INACTIVITY_EVENT_GYRO_SLEEP      = 0x40,
    ISM330DLC_INACTIVITY_EVENT_GYRO_POWER_DOWN = 0x60,

} ism330dlc_inactivity_event_mode_t;

/**
 * @brief Enables or disables the sensor to save power using inactivity mode
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param mode   Enables/Disables inactivity mode and determines what power mode the gyroscope goes into
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_inactivity_event_mode(ism330dlc_t *device, ism330dlc_inactivity_event_mode_t mode);

typedef enum
{
    ISM330DLC_EVENT_TAP_X_EN = 0x08,
    ISM330DLC_EVENT_TAP_Y_EN = 0x04,
    ISM330DLC_EVENT_TAP_Z_EN = 0x02,
} ism330dlc_tap_event_axis_t;

/**
 * @brief Enables or disables tap detection for a specific axis
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param axis   The axis to enable/disable (X, Y, or Z)
 * @param mode   Boolean to enable or disable tap detection on the axis
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_tap_event_axis_enable(ism330dlc_t *device, ism330dlc_tap_event_axis_t axis, bool enabled);

typedef enum
{
    ISM330DLC_INTERRUPT_UNLATCHED = 0x00,
    ISM330DLC_INTERRUPT_LATCHED
} ism330dlc_interrupt_latched_t;

/**
 * @brief Configures the interrupt latching behavior
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param mode The latching mode (Latched or Unlatched).
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_interrupt_latched_mode(ism330dlc_t *device, ism330dlc_interrupt_latched_t mode);

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

/**
 * @brief Routes a specific hardware event to a physical interrupt pin
 *
 * @param device Pointer to the ISM330DLC device instance.
 * @param pin    The target interrupt pin (INT1 or INT2).
 * @param event  The event to be routed (Wake-up, Free-fall, Tap, etc.)
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_event_interrupt_route(ism330dlc_t *device, ism330dlc_interrupt_pin_t pin, ism330dlc_md_event_t event, bool enabled);

typedef enum
{
    ISM330DLC_INACT_DISABLED      = 0x00,
    ISM330DLC_INACT_XL_12HZ       = 0x20,
    ISM330DLC_INACT_XL_12HZ_G_SLP = 0x40,
    ISM330DLC_INACT_XL_12HZ_G_PD  = 0x60,
} ism330dlc_inact_mode_t;

/**
 * @brief Converts from gs to the quantized 6-bit wake-up threshold
 *
 * @param device     Pointer to the ISM330DLC device instance.
 * @param accel_g    Desired threshold in units of g.
 * @param full_scale The current accelerometer full-scale range.
 *
 * @return Quantized 6-bit threshold value.
 */
uint8_t ism330dlc_convert_g_to_wakeup_threshold(float accel_g, ism330dlc_accel_full_scale_t full_scale);
/**
 * @brief Sets the raw wake-up threshold value
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param threshold Raw 6-bit threshold value.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_wakeup_threshold(ism330dlc_t *device, uint8_t threshold);

/**
 * @brief Converts from milliseconds to the quantized 2-bit wake-up duration
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param wakeup_ms Desired duration in units of milliseconds.
 * @param odr       The current accelerometer output data rate.
 *
 * @return Quantized 2-bit duration value.
 */
uint8_t ism330dlc_convert_ms_to_wakeup_dur(float wakeup_ms, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Sets the raw wake-up duration value
 *
 * @param device   Pointer to the ISM330DLC device instance.
 * @param duration Raw 2-bit threshold value.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_wakeup_duration(ism330dlc_t *device, uint8_t duration);

typedef enum
{
    ISM330DLC_FF_THRESHOLD_156_mg = 0x00,
    ISM330DLC_FF_THRESHOLD_219_mg = 0x01,
    ISM330DLC_FF_THRESHOLD_250_mg = 0x02,
    ISM330DLC_FF_THRESHOLD_312_mg = 0x03,
    ISM330DLC_FF_THRESHOLD_344_mg = 0x04,
    ISM330DLC_FF_THRESHOLD_406_mg = 0x05,
    ISM330DLC_FF_THRESHOLD_469_mg = 0x06,
    ISM330DLC_FF_THRESHOLD_500_mg = 0x05,
} ism330dlc_freefall_threshold_t;

/**
 * @brief Sets the threshold to trigger the freefall event interrupt
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param threshold The threshold to trigger the event.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_freefall_threshold(ism330dlc_t *device, ism330dlc_freefall_threshold_t threshold);

/**
 * @brief Sets the raw freefall duration value
 *
 * @param device   Pointer to the ISM330DLC device instance.
 * @param duration Raw 5-bit duration value.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_freefall_duration(ism330dlc_t *device, uint8_t duration);

/**
 * @brief Converts from g to the quantized 8-bit tap threshold value
 *
 * @param device     Pointer to the ISM330DLC device instance.
 * @param accel_g    Desired acceleration threshold in g.
 * @param full_scale The current accelerometer full scale.
 *
 * @return Quantized 8-bit threshold value.
 */
uint8_t ism330dlc_convert_g_to_tap_threshold(float accel_g, ism330dlc_accel_full_scale_t full_scale);

/**
 * @brief Sets the threshold to trigger the tap event
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param threshold Desired acceleration threshold in g.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_tap_threshold(ism330dlc_t *device, uint8_t threshold);

/**
 * @brief Converts from milliseconds to the quantized 4-bit double tap gap duration value
 *
 * @param gap_ms The target gap duration in milliseconds.
 * @param odr    The current accelerometer output data rate.
 *
 * @return Quantized 4-bit duration value.
 */
uint8_t ism330dlc_convert_ms_to_double_tap_gap_dur(float gap_ms, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Converts from milliseconds to the quantized 2-bit shock duration value
 *
 * @param shock_ms The target shock duration in milliseconds.
 * @param odr      The current accelerometer output data rate.
 *
 * @return Quantized 2-bit duration value.
 */
uint8_t ism330dlc_convert_ms_to_tap_shock_dur(float shock_ms, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Converts from milliseconds to the quantized 2-bit quiet duration value
 *
 * @param quiet_ms The target quiet duration in milliseconds.
 * @param odr      The current accelerometer output data rate.
 *
 * @return Quantized 2-bit duration value.
 */
uint8_t ism330dlc_convert_ms_to_tap_quiet_dur(float quiet_ms, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Sets the timing characteristics for tap recognition
 *
 * @param device   The target quiet duration in milliseconds.
 * @param shock    The raw 2-bit shock duration value
 * @param quiet    The raw 2-bit quiet duration value.
 * @param duration The raw 4-bit time gap for double tap recognition
 *
 * @return ISM330DLC Status Code.
 */
ism330dlc_status_t ism330dlc_set_tap_timing(ism330dlc_t *device, uint8_t shock, uint8_t quiet, uint8_t duration);

/**
 * @brief Enables or disables double tap detection 
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param enabled true enables both single and double-tap events; false enables only single-tap events
 *
 * @return ISM330DLC Status Code.
 */
ism330dlc_status_t ism330dlc_set_double_tap_enable(ism330dlc_t *device, bool enabled);

typedef enum
{
    ISM330DLC_6D_THS_80_DEG = 0x00,
    ISM330DLC_6D_THS_70_DEG = 0x20,
    ISM330DLC_6D_THS_60_DEG = 0x40,
    ISM330DLC_6D_THS_50_DEG = 0x60,
} ism330dlc_6d_threshold_t;

/**
 * @brief Enables or disables double tap detection 
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param enabled true enables both single and double-tap events; false enables only single-tap events
 *
 * @return ISM330DLC Status Code.
 */
ism330dlc_status_t ism330dlc_set_6d_threshold(ism330dlc_t *device, ism330dlc_6d_threshold_t threshold);

/**
 * @brief Enables or disables 4D orientation detection 
 *
 * @param device  Pointer to the ISM330DLC device instance.
 * @param enabled Whether the function is enabled or disabled
 *
 * @return ISM330DLC Status Code.
 */
ism330dlc_status_t ism330dlc_set_4d_orientation_enable(ism330dlc_t *device, bool enabled);

/**
 * @brief Converts from milliseconds to the quantized 4-bit sleep duration value
 *
 * @param quiet_ms The target quiet duration in milliseconds.
 * @param odr      The current accelerometer output data rate.
 *
 * @return Quantized 4-bit duration value.
 */
uint8_t ism330dlc_convert_ms_to_sleep_dur(float sleep_ms, ism330dlc_accel_gyro_odr_t odr);

/**
 * @brief Sets the threshold to trigger the tap event
 *
 * @param device    Pointer to the ISM330DLC device instance.
 * @param threshold Desired acceleration threshold in g.
 *
 * @return ISM330DLC status code.
 */
ism330dlc_status_t ism330dlc_set_sleep_duration(ism330dlc_t *device, uint8_t duration);

#endif
