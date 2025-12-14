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
    ISM330DLC_ADDR0 = 0x6A,
    ISM330DLC_ADDR1 = 0x6B
} ism330dlc_device_address_t;

typedef struct
{
    ism330dlc_bus_type_t bus_type;
    ism330dlc_status_t (*read_register)(void *handle, uint8_t registerAddress, uint8_t *buffer, size_t length);
    ism330dlc_status_t (*write_register)(void *handle, uint8_t registerAddress, uint8_t *data, size_t length);
} ism330dlc_bus_t;

ism330dlc_status_t ism330dlc_bus_init(
    ism330dlc_bus_t *bus, 
    ism330dlc_status_t (*read_register)(void*, uint8_t, uint8_t*, size_t),
    ism330dlc_status_t (*write_register)(void*, uint8_t, uint8_t*, size_t)
);

typedef struct
{
    ism330dlc_bus_t* bus_config;
    void* platform_handle;
} ism330dlc_t;

void ism330dlc_init(ism330dlc_t *instance, const ism330dlc_bus_t *bus);

/**
 * Register : WHO_AM_I
 * Address  : 0x0F
 *
 */
ism330dlc_status_t ism330dlc_read_who_am_i(uint8_t *result);

typedef enum
{
    ISM330DLC_OUTPUT_TEMP_L = 0x20,
    ISM330DLC_OUTPUT_TEMP_R = 0x21
} ISM330DLC_OUT_TEMP_t;

#endif
