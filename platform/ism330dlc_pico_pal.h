#ifndef ISM330DLC_PICO_PAL_H
#define ISM330DLC_PICO_PAL_H

#define ISM330DLC_SPI3_DISABLED

#include "ism330dlc_driver/ism330dlc_driver.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

typedef struct
{
    i2c_inst_t *port;
    ism330dlc_i2c_address_t i2c_address;
    uint baudrate;
    uint8_t cs_pin;
    uint8_t scl_pin;
    uint8_t sda_pin;
} ism330dlc_pico_i2c_config;

ism330dlc_status_t ism330dlc_pico_i2c_bus_init(const ism330dlc_pico_i2c_config config);
ism330dlc_status_t ism330dlc_pico_i2c_pins_init(const ism330dlc_pico_i2c_config config);
ism330dlc_status_t ism330dlc_pico_i2c_bus_deinit(const ism330dlc_pico_i2c_config config);
ism330dlc_status_t ism330dlc_pico_i2c_pins_deinit(const ism330dlc_pico_i2c_config config);

typedef struct
{
    spi_inst_t *port;
    uint baudrate;
    uint8_t cs_pin;
    uint8_t scl_pin;
    uint8_t sdo_pin;
    uint8_t sdi_pin;
} ism330dlc_pico_spi4_config;

ism330dlc_status_t ism330dlc_pico_spi4_bus_init(const ism330dlc_pico_spi4_config config);
ism330dlc_status_t ism330dlc_pico_spi4_pins_init(const ism330dlc_pico_spi4_config config);
ism330dlc_status_t ism330dlc_pico_spi4_bus_deinit(const ism330dlc_pico_spi4_config config);
ism330dlc_status_t ism330dlc_pico_spi4_pins_deinit(const ism330dlc_pico_spi4_config config);
#endif