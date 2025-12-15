#include "ism330dlc_driver.h"
#include "hardware/i2c.h"
#include "ism330dlc_regs.h"

void ism330dlc_init(
    ism330dlc_t *instance, 
    void* device_context, 
    ism330dlc_status_t (*read_registers)(void *device_context, uint8_t registerAddress, uint8_t *buffer, size_t length),
    ism330dlc_status_t (*write_registers)(void *device_context, uint8_t registerAddress, uint8_t *data, size_t length)
) 
{
    instance->device_context = device_context;
    instance->read_registers = read_registers;
    instance->write_registers = write_registers;
};

ism330dlc_status_t ism330dlc_read_who_am_i(ism330dlc_t* device, uint8_t* result) 
{
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_WHO_AM_I,
        result,
        1
    );
};

ism330dlc_status_t ism330dlc_read_raw_accel_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result) {
    ism330dlc_status_t response = device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUTX_L_XL,
        result->values,
        6
    );
    return ISM330DLC_SUCCESS;
};


ism330dlc_status_t ism330dlc_read_raw_gyro_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result) {
    ism330dlc_status_t response = device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUTX_L_G,
        result->values,
        6
    );
    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_read_raw_temperature_data(ism330dlc_t* device, ism330dlc_reg16_t *result)
{
    ism330dlc_status_t response = device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUT_TEMP_L,
        result->u8,
        2
    );
    return ISM330DLC_SUCCESS;    
};

