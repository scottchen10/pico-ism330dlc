#ifndef ISM330DLC_PICO_HAL
#define ISM330DLC_PICO_HAL

#include "ism330dlc_driver.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

typedef struct
{
    i2c_inst_t *port;
    ism330dlc_device_address_t i2c_address;
    uint baudrate;
    uint8_t cs_pin;
    uint8_t scl_pin;
    uint8_t sda_pin;
} ism330dlc_pico_i2c_config;

ism330dlc_status_t ism330dlc_pico_i2c_bus_init(ism330dlc_pico_i2c_config *config);
ism330dlc_status_t ism330dlc_pico_i2c_pins_init(ism330dlc_pico_i2c_config *config);
ism330dlc_status_t ism330dlc_pico_i2c_bus_deinit(ism330dlc_pico_i2c_config *config);
ism330dlc_status_t ism330dlc_pico_i2c_pins_deinit(ism330dlc_pico_i2c_config *config);

ism330dlc_status_t i2c_read_register(void *handle, uint8_t registerAddress, uint8_t *buffer, size_t length);
ism330dlc_status_t i2c_write_register(void *handle, uint8_t registerAddress, uint8_t *data, size_t length) ;

typedef struct
{
    spi_inst_t *port;
    uint baudrate;
    uint8_t cs_pin;
    uint8_t scl_pin;
    uint8_t sdo_pin;
    uint8_t sdi_pin;
} ism330dlc_pico_spi_config;

ism330dlc_status_t ism330dlc_pico_spi_bus_init(ism330dlc_pico_spi_config *config);
ism330dlc_status_t ism330dlc_pico_spi_pins_init(ism330dlc_pico_spi_config *config);
ism330dlc_status_t ism330dlc_pico_spi_bus_deinit(ism330dlc_pico_spi_config *config);
ism330dlc_status_t ism330dlc_pico_spi_pins_deinit(ism330dlc_pico_spi_config *config);

ism330dlc_status_t spi_read_register(void *handle, uint8_t registerAddress, uint8_t *buffer, size_t length);
ism330dlc_status_t spi_write_register(void *handle, uint8_t registerAddress, uint8_t *data, size_t length) ;

#endif