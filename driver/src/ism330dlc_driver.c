#include "ism330dlc_driver.h"
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
    uint8_t ctrl6_c_reg_state; 
    
    ism330dlc_status_t resp = device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_CTRL6_C,
        &ctrl6_c_reg_state,
        1
    );

    if (resp != ISM330DLC_SUCCESS) 
        return resp;

    uint8_t ctrl_c_new_state = ctrl6_c_reg_state & ~ISM330DLC_MASK_XL_HM_MODE;
    
    if (mode == ISM330DLC_ACCEL_GYRO_LOW_PERFORMANCE) 
    {
        ctrl_c_new_state |= ISM330DLC_MASK_XL_HM_MODE;
    }

    return device->write_registers(
        device->device_context,
        ISM330DLC_ADDR_CTRL6_C,
        &ctrl_c_new_state,
        1
    );
};
ism330dlc_status_t ism330dlc_update_accel_odr(ism330dlc_t* device, ism330dlc_accel_gyro_odr_t odr);
ism330dlc_status_t ism330dlc_read_accel_odr(ism330dlc_t* device);

ism330dlc_status_t ism330dlc_update_gyro_performance_mode(ism330dlc_t* device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    uint8_t ctrl7_c_reg_state; 
    
    ism330dlc_status_t resp = device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_CTRL7_G,
        &ctrl7_c_reg_state,
        1
    );

    if (resp != ISM330DLC_SUCCESS) 
        return resp;

    uint8_t ctrl_c_new_state = ctrl7_c_reg_state & ~ISM330DLC_MASK_G_HM_MODE;
    
    if (mode == ISM330DLC_ACCEL_GYRO_LOW_PERFORMANCE) 
    {
        ctrl_c_new_state |= ISM330DLC_MASK_G_HM_MODE;
    }

    return device->write_registers(
        device->device_context,
        ISM330DLC_ADDR_CTRL7_G,
        &ctrl_c_new_state,
        1
    );
};

ism330dlc_status_t ism330dlc_update_gyro_odr(ism330dlc_t* device, ism330dlc_accel_gyro_odr_t odr);
ism330dlc_status_t ism330dlc_read_gyro_odr(ism330dlc_t* device);