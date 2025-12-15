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
 * @return status [ISM330DLC_SUCCESS, ISM330DLC_ERROR]
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
 * @brief Reads the raw accelerometer data from registers OUTX_L_XL to OUTZ_H_XL
 * 
 * @param result Pointer to where the raw xyz accelerometer data will be stored
 * @return status [ISM330DLC_SUCCESS, ISM330DLC_ERROR]
 */
ism330dlc_status_t ism330dlc_read_raw_accel_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result);

/**
 * @brief Reads the raw gyro data from registers OUTX_L_G to OUTZ_H_G
 * 
 * @param result Pointer to where the raw xyz gyroscope data will be stored
 * @return status [ISM330DLC_SUCCESS, ISM330DLC_ERROR]
 */
ism330dlc_status_t ism330dlc_read_raw_gyro_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result);

/**
 * @brief Reads the raw temperature data from registers OUT_TEMP_L to OUT_TEMP_H
 * 
 * @param result Pointer to where the raw temperature data will be stored
 * @return status [ISM330DLC_SUCCESS, ISM330DLC_ERROR]
 */
ism330dlc_status_t ism330dlc_read_raw_temperature_data(ism330dlc_t* device, ism330dlc_reg16_t *result);




#endif
