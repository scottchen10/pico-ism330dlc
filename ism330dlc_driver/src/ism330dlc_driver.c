#include "ism330dlc_driver/ism330dlc_driver.h"
#include "ism330dlc_driver/ism330dlc_regs.h"

#include "math.h"

static ism330dlc_status_t ism330dlc_write_register_with_mask(
    ism330dlc_t *device,
    uint8_t address,
    uint8_t reset_mask,
    uint8_t masked_value)
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
    ism330dlc_t *device,
    uint8_t address,
    uint8_t mask,
    uint8_t *value
)
{
    ism330dlc_status_t resp = device->read_registers(
        device->device_context,
        address,
        value,
        1
    );

    (*value) &= mask;

    return resp;
};

ism330dlc_init_status ism330dlc_init(
    ism330dlc_t *instance,
    void *device_context,
    ism330dlc_bus_type_t bus_type)
{
    instance->device_context = device_context;
    instance->last_gyro_fs = ISM330DLC_GYRO_FS_250DPS;
    instance->last_accel_fs = ISM330DLC_ACCEL_FS_2G;

    switch (bus_type)
    {
    case ISM330DLC_BUS_I2C:
        instance->read_registers = &ism330dlc_i2c_read_registers;
        instance->write_registers = &ism330dlc_i2c_write_registers;

        break;
    case ISM330DLC_BUS_SPI4:
        instance->read_registers = &ism330dlc_spi4_write_registers;
        instance->write_registers = &ism330dlc_spi4_read_registers;
        break;
    case ISM330DLC_BUS_SPI3:
#ifdef ISM330DLC_SPI3_DISABLED
        return ISM330DLC_INIT_FAIL_SPI3_DISABLED;
#endif  
#ifndef ISM330DLC_SPI3_DISABLED 
        instance->read_registers = &ism330dlc_spi3_write_registers;
        instance->write_registers = &ism330dlc_spi3_read_registers;
        break;
#endif
    default:
        return ISM330DLC_INIT_FAIL_INVALID_BUS;
    }

    return ISM330DLC_INIT_SUCCESS;
};

ism330dlc_status_t ism330dlc_read_who_am_i(ism330dlc_t *device, uint8_t *result)
{
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_WHO_AM_I,
        result,
        1
    );
};

ism330dlc_status_t ism330dlc_read_raw_accel_data(ism330dlc_t *device, ism330dlc_raw_xyz_t *result)
{
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUTX_L_XL,
        result->bytes,
        6
    );
    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_read_raw_gyro_data(ism330dlc_t *device, ism330dlc_raw_xyz_t *result)
{
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUTX_L_G,
        result->bytes,
        6
    );
    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_read_raw_temperature_data(ism330dlc_t *device, ism330dlc_reg16_t *result)
{
    return device->read_registers(
        device->device_context,
        ISM330DLC_ADDR_OUT_TEMP_L,
        result->bytes,
        2
    );
};

ism330dlc_status_t ism330dlc_update_accel_performance_mode(ism330dlc_t *device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL6_C,
        ISM330DLC_MASK_XL_HM_MODE,
        (mode == ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE) ? 0x00 : ISM330DLC_MASK_XL_HM_MODE
    );
};
ism330dlc_status_t ism330dlc_update_accel_odr(ism330dlc_t *device, ism330dlc_accel_gyro_odr_t odr)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_ODR_XL,
        odr
    );
};

ism330dlc_status_t ism330dlc_update_gyro_performance_mode(ism330dlc_t *device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL7_G,
        ISM330DLC_MASK_G_HM_MODE,
        (mode == ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE) ? 0x00 : ISM330DLC_MASK_G_HM_MODE
    );
};

ism330dlc_status_t ism330dlc_update_gyro_odr(ism330dlc_t *device, ism330dlc_accel_gyro_odr_t odr)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_ODR_G,
        odr
    );
};

ism330dlc_status_t ism330dlc_update_accel_full_scale(ism330dlc_t *device, ism330dlc_accel_full_scale_t scale)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_FS_XL,
        scale
    );
};

ism330dlc_status_t ism330dlc_read_accel_full_scale(ism330dlc_t *device, ism330dlc_accel_full_scale_t *scale)
{
    uint8_t value;

    ism330dlc_status_t resp = ism330dlc_read_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_FS_XL,
        &value
    );

    (*scale) = (ism330dlc_accel_full_scale_t)value;

    if (resp == ISM330DLC_SUCCESS)
        device->last_accel_fs = *scale;

    return resp;
};

ism330dlc_status_t ism330dlc_update_gyro_full_scale(ism330dlc_t *device, ism330dlc_gyro_full_scale_t scale)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_FS_G,
        scale
    );
};

ism330dlc_status_t ism330dlc_read_gyro_full_scale(ism330dlc_t *device, ism330dlc_gyro_full_scale_t *scale)
{
    uint8_t value;

    ism330dlc_status_t resp = ism330dlc_read_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_FS_G,
        &value
    );

    (*scale) = (ism330dlc_gyro_full_scale_t)value;

    if (resp == ISM330DLC_SUCCESS)
        device->last_gyro_fs = *scale;

    return resp;
};


static inline float ism330dlc_convert_raw_accel_to_g(int16_t raw_accel, ism330dlc_accel_full_scale_t scale)
{
    switch (scale)
    {
    case ISM330DLC_ACCEL_FS_2G:
        return raw_accel * ISM330DLC_SENS_ACCEL_FS_2G;
    case ISM330DLC_ACCEL_FS_4G:
        return raw_accel * ISM330DLC_SENS_ACCEL_FS_4G;
    case ISM330DLC_ACCEL_FS_8G:
        return raw_accel * ISM330DLC_SENS_ACCEL_FS_8G;
    case ISM330DLC_ACCEL_FS_16G:
        return raw_accel * ISM330DLC_SENS_ACCEL_FS_16G;
    }

    return NAN;
};

static inline float ism330dlc_convert_raw_accel_to_mps2(int16_t raw_accel, ism330dlc_accel_full_scale_t scale)
{
    return ism330dlc_convert_raw_accel_to_g(raw_accel, scale) * ISM330DLC_CONV_G_TO_MPS;
};

static inline float ism330dlc_convert_raw_gyro_to_dps(int16_t raw_gyro, ism330dlc_gyro_full_scale_t scale) 
{
    switch (scale)
    {
    case ISM330DLC_GYRO_FS_125DPS:
        return raw_gyro * ISM330DLC_SENS_GYRO_FS_125DPS;
    case ISM330DLC_GYRO_FS_250DPS:
        return raw_gyro * ISM330DLC_SENS_GYRO_FS_250DPS;
    case ISM330DLC_GYRO_FS_500DPS:
        return raw_gyro * ISM330DLC_SENS_GYRO_FS_500DPS;
    case ISM330DLC_GYRO_FS_1000DPS:
        return raw_gyro * ISM330DLC_SENS_GYRO_FS_1000DPS;
    case ISM330DLC_GYRO_FS_2000DPS:
        return raw_gyro * ISM330DLC_SENS_GYRO_FS_2000DPS;
    }

    return NAN;    
};

static inline float ism330dlc_convert_raw_gyro_to_rps(int16_t raw_gyro, ism330dlc_gyro_full_scale_t scale)
{
    return ism330dlc_convert_raw_gyro_to_dps(raw_gyro, scale) * ISM330DLC_CONV_DPS_TO_RPS;
};


void ism330dlc_convert_raw_accel_xyz_to_g(
    ism330dlc_accel_full_scale_t scale, 
    const ism330dlc_raw_xyz_t *raw_values, 
    ism330dlc_accel_t *accel_g
)
{
    accel_g->x = ism330dlc_convert_raw_accel_to_g(raw_values->axes[0], scale);
    accel_g->y = ism330dlc_convert_raw_accel_to_g(raw_values->axes[1], scale);
    accel_g->z = ism330dlc_convert_raw_accel_to_g(raw_values->axes[2], scale);
};

void ism330dlc_convert_raw_accel_xyz_to_mps2(
    ism330dlc_accel_full_scale_t scale, 
    const ism330dlc_raw_xyz_t *raw_values, 
    ism330dlc_accel_t *accel_mps
)
{
    accel_mps->x = ism330dlc_convert_raw_accel_to_mps2(raw_values->axes[0], scale);
    accel_mps->y = ism330dlc_convert_raw_accel_to_mps2(raw_values->axes[1], scale);
    accel_mps->z = ism330dlc_convert_raw_accel_to_mps2(raw_values->axes[2], scale);    
};

void ism330dlc_convert_raw_gyro_xyz_to_dps(
    ism330dlc_gyro_full_scale_t scale, 
    const ism330dlc_raw_xyz_t *raw_values, 
    ism330dlc_gyro_t *gyro_dps
)
{
    gyro_dps->x = ism330dlc_convert_raw_gyro_to_dps(raw_values->axes[0], scale);
    gyro_dps->y = ism330dlc_convert_raw_gyro_to_dps(raw_values->axes[1], scale);
    gyro_dps->z = ism330dlc_convert_raw_gyro_to_dps(raw_values->axes[2], scale);        
};

void ism330dlc_convert_raw_gyro_xyz_to_rps(
    ism330dlc_gyro_full_scale_t scale, 
    const ism330dlc_raw_xyz_t *raw_values, 
    ism330dlc_gyro_t *gyro_rps
)
{   
    gyro_rps->x = ism330dlc_convert_raw_gyro_to_rps(raw_values->axes[0], scale);
    gyro_rps->y = ism330dlc_convert_raw_gyro_to_rps(raw_values->axes[1], scale);
    gyro_rps->z = ism330dlc_convert_raw_gyro_to_rps(raw_values->axes[2], scale);        
};

float ism330dlc_convert_raw_temp_to_celcius(int16_t raw_temp)
{
    return ISM330DLC_TEMPERATURE_OFFSET + raw_temp * ISM330DLC_SENS_TEMP;
};
