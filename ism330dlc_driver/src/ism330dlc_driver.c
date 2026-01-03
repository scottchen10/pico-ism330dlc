#include "ism330dlc/ism330dlc_driver.h"
#include "ism330dlc/ism330dlc_regs.h"

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

    uint8_t new_register_state = (register_state & ~reset_mask) | (masked_value & ~reset_mask);

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

ism330dlc_status_t ism330dlc_set_accel_performance_mode(ism330dlc_t *device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL6_C,
        ISM330DLC_MASK_XL_HM_MODE,
        (mode == ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE) ? 0x00 : ISM330DLC_MASK_XL_HM_MODE
    );
};
ism330dlc_status_t ism330dlc_set_accel_odr(ism330dlc_t *device, ism330dlc_accel_gyro_odr_t odr)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL1_XL,
        ISM330DLC_MASK_ODR_XL,
        odr
    );
};

ism330dlc_status_t ism330dlc_set_gyro_performance_mode(ism330dlc_t *device, ism330dlc_accel_gyro_performance_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL7_G,
        ISM330DLC_MASK_G_HM_MODE,
        (mode == ISM330DLC_ACCEL_GYRO_HIGH_PERFORMANCE) ? 0x00 : ISM330DLC_MASK_G_HM_MODE
    );
};

ism330dlc_status_t ism330dlc_set_gyro_odr(ism330dlc_t *device, ism330dlc_accel_gyro_odr_t odr)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL2_G,
        ISM330DLC_MASK_ODR_G,
        odr
    );
};

ism330dlc_status_t ism330dlc_set_accel_full_scale(ism330dlc_t *device, ism330dlc_accel_full_scale_t scale)
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

ism330dlc_status_t ism330dlc_set_gyro_full_scale(ism330dlc_t *device, ism330dlc_gyro_full_scale_t scale)
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

ism330dlc_status_t ism330dlc_set_interrupt_active_mode(ism330dlc_t *device, ism330dlc_interrupt_active_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL3_C,
        ISM330DLC_MASK_H_LACTIVE,
        (uint8_t)mode
    );
};

ism330dlc_status_t ism330dlc_set_interrupt_output_mode(ism330dlc_t *device, ism330dlc_interrupt_output_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_CTRL3_C,
        ISM330DLC_MASK_PP_OD,
        (uint8_t)mode
    );
};


ism330dlc_status_t ism330dlc_set_event_interrupts_enable(ism330dlc_t *device, bool are_event_driven_interrupts_enabled)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_CFG,
        ISM330DLC_MASK_INTERRUPTS_EN,
        are_event_driven_interrupts_enabled ? ISM330DLC_MASK_INTERRUPTS_EN : 0x00
    );
};

ism330dlc_status_t ism330dlc_set_inactivity_event_mode(ism330dlc_t *device, ism330dlc_inactivity_event_mode_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_CFG,
        ISM330DLC_MASK_INACT_EN,
        (uint8_t)mode
    );
};
ism330dlc_status_t ism330dlc_set_tap_event_axis_enable(ism330dlc_t *device, ism330dlc_tap_event_axis_t axis, bool enabled)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_CFG,
        (uint8_t)axis,
        enabled ? (uint8_t)axis : 0x00
    );
};

ism330dlc_status_t ism330dlc_set_interrupt_latched(ism330dlc_t *device, ism330dlc_interrupt_latched_t mode)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_CFG,
        ISM330DLC_MASK_LIR,
        (uint8_t)mode
    );
};

ism330dlc_status_t ism330dlc_set_event_interrupt_route(ism330dlc_t *device, ism330dlc_interrupt_pin_t pin, ism330dlc_md_event_t event, bool enabled)
{
    uint8_t reg_addr = (pin == ISM330DLC_INTERRUPT_PIN_1) ? ISM330DLC_ADDR_MD1_CFG : ISM330DLC_ADDR_MD2_CFG;
    
    return ism330dlc_write_register_with_mask(
        device,
        reg_addr,
        (uint8_t)event,
        enabled ? (uint8_t)event : 0x00
    );    
};

static uint8_t ism330dlc_quantize(float x, float lsb_value, uint8_t max_bits_value)
{
    int32_t quantized_value = (int32_t)(x/lsb_value);

    if (quantized_value < 0) quantized_value = 0;
    if (quantized_value > max_bits_value) quantized_value = max_bits_value;
    return (uint8_t)quantized_value;    
}

uint8_t ism330dlc_convert_g_to_wakeup_threshold(float accel_g, ism330dlc_accel_full_scale_t full_scale)
{
    float lsb_value;
    switch (full_scale) {
        case ISM330DLC_ACCEL_FS_2G : lsb_value = ISM330DLC_SENS_WAKEUP_THS_2G;  break;
        case ISM330DLC_ACCEL_FS_4G : lsb_value = ISM330DLC_SENS_WAKEUP_THS_4G;  break;
        case ISM330DLC_ACCEL_FS_8G : lsb_value = ISM330DLC_SENS_WAKEUP_THS_8G;  break;
        default:                     lsb_value = ISM330DLC_SENS_WAKEUP_THS_16G; break;
    }
    
    return ism330dlc_quantize(accel_g, lsb_value, 255);
};

ism330dlc_status_t ism330dlc_set_wakeup_threshold(ism330dlc_t *device, uint8_t threshold)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_WAKE_UP_THS,
        ISM330DLC_MASK_WK_THS,
        threshold
    );
};

ism330dlc_status_t ism330dlc_set_freefall_threshold(ism330dlc_t *device, ism330dlc_freefall_threshold_t threshold)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_FREE_FALL,
        ISM330DLC_MASK_FF_THS,
        threshold
    );    
};

uint8_t ism330dlc_convert_g_to_tap_threshold(float accel_g, ism330dlc_accel_full_scale_t full_scale)
{
    float lsb_value;
    switch (full_scale) {
        case ISM330DLC_ACCEL_FS_2G : lsb_value = ISM330DLC_SENS_TAP_THS_2G;  break;
        case ISM330DLC_ACCEL_FS_4G : lsb_value = ISM330DLC_SENS_TAP_THS_4G;  break;
        case ISM330DLC_ACCEL_FS_8G : lsb_value = ISM330DLC_SENS_TAP_THS_8G;  break;
        default:                     lsb_value = ISM330DLC_SENS_TAP_THS_16G; break;
    }

    return ism330dlc_quantize(accel_g, lsb_value, 255);
};

ism330dlc_status_t ism330dlc_set_tap_threshold(ism330dlc_t *device, uint8_t threshold)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_THS_6D,
        ISM330DLC_MASK_TAP_THS,
        threshold
    );
};


ism330dlc_status_t ism330dlc_set_double_tap_enable(ism330dlc_t *device, bool enabled)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_WAKE_UP_THS,
        ISM330DLC_MASK_SINGLE_DBL_TAP,
        enabled ? ISM330DLC_MASK_SINGLE_DBL_TAP : 0x00
    );
};

ism330dlc_status_t ism330dlc_set_6d_threshold(ism330dlc_t *device, ism330dlc_6d_threshold_t threshold)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_THS_6D,
        ISM330DLC_MASK_6D_THS,
        (uint8_t)threshold
    );
};
ism330dlc_status_t ism330dlc_set_4d_orientation_enable(ism330dlc_t *device, bool enabled)
{
    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_TAP_THS_6D,
        ISM330DLC_MASK_D4D_EN,
        enabled ? ISM330DLC_MASK_D4D_EN : 0x00
    );    
};

static float ism330dlc_get_odr_period(ism330dlc_accel_gyro_odr_t odr)
{
    switch (odr)
    {
        case ISM330DLC_ACCEL_GYRO_ODR_1_6_HZ     : return 1/1.6f;
        case ISM330DLC_ACCEL_GYRO_ODR_12_5_HZ    : return 1/12.5f;
        case ISM330DLC_ACCEL_GYRO_ODR_26_HZ      : return 1/26.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_52_HZ      : return 1/52.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_104_HZ     : return 1/104.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_208_HZ     : return 1/208.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_416_HZ     : return 1/416.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_833_HZ     : return 1/833.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_1660_HZ    : return 1/1660.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_3330_HZ    : return 1/3330.0f;
        case ISM330DLC_ACCEL_GYRO_ODR_6660_HZ    : return 1/6660.0f;
        default                                  : return 0.0f;
    }
}

uint8_t ism330dlc_convert_ms_to_freefall_dur(float ff_ms, ism330dlc_accel_gyro_odr_t odr)
{
    float odr_hz = ism330dlc_get_odr_hz(odr);
    if (odr_hz <= 0.0f) return 0;

    float lsb_ms = 1000.0f / odr_hz;
    return ism330dlc_quantize(ff_ms, lsb_ms, 63);
}


ism330dlc_status_t ism330dlc_set_freefall_duration(ism330dlc_t *device, uint8_t duration)
{
    /* FF_DUR[5] is Bit 7 of WAKE_UP_DUR (5Ch)
     * FF_DUR[4:0] is Bits [7:3] of FREE_FALL (5Dh)
     */
    ism330dlc_status_t status;

    uint8_t ff_dur_msb = (duration & ISM330DLC_MASK_FF_RAW_MSB) ? ISM330DLC_MASK_FF_DUR5 : 0x00;
    status = ism330dlc_write_register_with_mask(
        device, 
        ISM330DLC_ADDR_WAKE_UP_DUR, 
        ISM330DLC_MASK_FF_DUR5, 
        ff_dur_msb
    );
    if (status != ISM330DLC_SUCCESS) return status;

    uint8_t ff_dur_lsbs = duration << 3;
    return ism330dlc_write_register_with_mask(
        device, 
        ISM330DLC_ADDR_FREE_FALL, 
        ISM330DLC_MASK_FF_DUR_4_0, 
        ff_dur_lsbs
    );
};

uint8_t ism330dlc_convert_ms_to_wakeup_dur(float wakeup_ms, ism330dlc_accel_gyro_odr_t odr)
{
    float odr_hz = ism330dlc_get_odr_hz(odr);
    if (odr_hz <= 0.0f) return 0;

    float lsb_ms = 1000.0f / odr_hz;
    return ism330dlc_quantize(wakeup_ms, lsb_ms, 3);
}

ism330dlc_status_t ism330dlc_set_wakeup_duration(ism330dlc_t *device, uint8_t duration)
{
    return ism330dlc_write_register_with_mask(
        device, 
        ISM330DLC_ADDR_WAKE_UP_DUR, 
        ISM330DLC_MASK_WAKE_DUR, 
        (duration << 5)
    );
};

uint8_t ism330dlc_convert_ms_to_double_tap_gap_dur(float gap_ms, ism330dlc_accel_gyro_odr_t odr)
{
    float odr_period_sec = ism330dlc_get_odr_period(odr);
    if (odr_period_sec <= 0.0f) return 0;

    float lsb_ms = 32000.0f * odr_period_sec;
    return ism330dlc_quantize(gap_ms, lsb_ms, 15);
}

uint8_t ism330dlc_convert_ms_to_tap_shock_dur(float shock_ms, ism330dlc_accel_gyro_odr_t odr)
{
    float odr_period_sec = ism330dlc_get_odr_period(odr);
    if (odr_period_sec <= 0.0f) return 0;

    float lsb_ms = 8000.0f * odr_period_sec;
    return ism330dlc_quantize(shock_ms, lsb_ms, 3);
}

uint8_t ism330dlc_convert_ms_to_tap_quiet_dur(float quiet_ms, ism330dlc_accel_gyro_odr_t odr)
{
    float odr_period_sec = ism330dlc_get_odr_period(odr);
    if (odr_period_sec <= 0.0f) return 0;

    float lsb_ms = 4000.0f * odr_period_sec;
    return ism330dlc_quantize(quiet_ms, lsb_ms, 3);
}

ism330dlc_status_t ism330dlc_set_tap_timing(ism330dlc_t *device, uint8_t shock, uint8_t quiet, uint8_t duration)
{
    /* * INT_DUR2 (5Ah) Bit Map:
     * [7:4] DUR (duration)
     * [3:2] QUIET (quiet)
     * [1:0] SHOCK (shock)
     */
    uint8_t value = 0;

    value |= (duration << 4) & ISM330DLC_MASK_DUR;
    value |= (quiet << 2)    & ISM330DLC_MASK_QUIET;
    value |= (shock)         & ISM330DLC_MASK_SHOCK;

    return ism330dlc_write_register_with_mask(
        device,
        ISM330DLC_ADDR_INT_DUR2,
        0xFF,
        value
    );
};

uint8_t ism330dlc_convert_ms_to_sleep_dur(float sleep_ms, ism330dlc_accel_gyro_odr_t odr)
{
    float odr_hz = ism330dlc_get_odr_hz(odr);
    if (odr_hz <= 0.0f) return 0;

    float lsb_ms = 512000.0f / odr_hz;
    
    return ism330dlc_quantize(sleep_ms, lsb_ms, 15);
}

ism330dlc_status_t ism330dlc_set_sleep_duration(ism330dlc_t *device, uint8_t duration)
{
    return ism330dlc_write_register_with_mask(
        device, 
        ISM330DLC_ADDR_WAKE_UP_DUR, 
        ISM330DLC_MASK_SLEEP_DUR, 
        duration
    );    
};