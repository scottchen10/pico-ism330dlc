#include "ism330dlc_driver.h"
#include "ism330dlc_regs.h"

static ism330dlc_status_t ism330dlc_write_register_with_mask(
    ism330dlc_t* device,
    uint8_t address,
    uint8_t reset_mask,
    uint8_t masked_value
)
{
    uint8_t register_state; 
    
    ism330dlc_status_t resp = device->read_registers(
        device->device_context,
        address,
        &register_state,
        1
    );

    if (resp != ISM330DLC_SUCCESS) 
        return resp;

    uint8_t new_register_state = (register_state & ~reset_mask) | masked_value;

    return device->write_registers(
        device->device_context,
        address,
        &new_register_state,
        1
    );
};

static ism330dlc_status_t ism330dlc_read_register_with_mask(
    ism330dlc_t* device,
    uint8_t address,
    uint8_t reset_mask,
    uint8_t* value
)
{    
    ism330dlc_status_t resp =  device->read_registers(
        device->device_context,
        address,
        value,
        1
    );

    (*value) &= ~reset_mask;
    return resp;
};

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
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUTX_L_XL,
        result->values,
        6
    );
    return ISM330DLC_SUCCESS;
};


ism330dlc_status_t ism330dlc_read_raw_gyro_data(ism330dlc_t* device, ism330dlc_raw_xyz_t *result) {
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUTX_L_G,
        result->values,
        6
    );
    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_read_raw_temperature_data(ism330dlc_t* device, ism330dlc_reg16_t *result)
{
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUT_TEMP_L,
        result->u8,
        2
    );
};

ism330dlc_status_t ism330dlc_update_accel_performance_mode(ism330dlc_t* device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL6_C,
        ISM330DLC_MASK_XL_HM_MODE,
        (mode == ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE) ? 0x00: ISM330DLC_MASK_XL_HM_MODE
    );
};
ism330dlc_status_t ism330dlc_update_accel_odr(ism330dlc_t* device, ism330dlc_accel_gyro_odr_t odr)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_ODR_XL,
        odr
    );
};

ism330dlc_status_t ism330dlc_update_gyro_performance_mode(ism330dlc_t* device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL7_G,
        ISM330DLC_MASK_G_HM_MODE,
        (mode == ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE) ? 0x00: ISM330DLC_MASK_G_HM_MODE
    );
};

ism330dlc_status_t ism330dlc_update_gyro_odr(ism330dlc_t* device, ism330dlc_accel_gyro_odr_t odr)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_ODR_G,
        odr
    );    
};

ism330dlc_status_t ism330dlc_update_accel_full_scale(ism330dlc_t* device, ism330dlc_accel_full_scale_t scale)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_FS_XL,
        scale
    );    
};

ism330dlc_status_t ism330dlc_read_accel_full_scale(ism330dlc_t* device, ism330dlc_accel_full_scale_t* scale) 
{
    uint8_t value;
    
    ism330dlc_status_t resp = ism330dlc_read_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_FS_XL,
        &value
    );

    (*scale) = (ism330dlc_accel_full_scale_t) value;

    if (resp == ISM330DLC_SUCCESS)
        device->last_accel_fs = *scale; 

    return resp;
};

ism330dlc_status_t ism330dlc_update_gyro_full_scale(ism330dlc_t* device, ism330dlc_gyro_full_scale_t scale)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_FS_G,
        scale
    );    
};

ism330dlc_status_t ism330dlc_read_gyro_full_scale(ism330dlc_t* device, ism330dlc_gyro_full_scale_t* scale)
{
    uint8_t value;
    
    ism330dlc_status_t resp = ism330dlc_read_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_FS_G,
        &value
    );

    (*scale) = (ism330dlc_accel_full_scale_t) value;

    if (resp == ISM330DLC_SUCCESS)
        device->last_gyro_fs = *scale; 

    return resp;
};





