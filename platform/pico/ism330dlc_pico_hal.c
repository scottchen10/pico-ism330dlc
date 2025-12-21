#include "ism330dlc_pico_hal.h"
#include "hardware/gpio.h"
#include <assert.h>

ism330dlc_status_t ism330dlc_pico_i2c_bus_init(const ism330dlc_pico_i2c_config config)
{
    (void)i2c_init(config.port, config.baudrate);
    gpio_set_function(config.sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config.scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(config.scl_pin);
    gpio_pull_up(config.sda_pin);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_pico_i2c_pins_init(const ism330dlc_pico_i2c_config config)
{
    gpio_init(config.cs_pin);
    gpio_set_dir(config.cs_pin, GPIO_OUT);
    gpio_put(config.cs_pin, 1);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_pico_i2c_bus_deinit(const ism330dlc_pico_i2c_config config)
{
    i2c_deinit(config.port);
    gpio_set_function(config.scl_pin, GPIO_FUNC_SIO);
    gpio_set_function(config.sda_pin, GPIO_FUNC_SIO);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_pico_i2c_pins_deinit(const ism330dlc_pico_i2c_config config)
{
    gpio_deinit(config.cs_pin);
    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_i2c_read_registers(void *handle, uint8_t address, uint8_t *buffer, size_t length)
{
    ism330dlc_pico_i2c_config *config = (ism330dlc_pico_i2c_config *)handle;
    int resp;
    resp = i2c_write_blocking(config->port, config->i2c_address, &address, sizeof(address), true);

    if (resp == PICO_ERROR_GENERIC)
        return ISM330DLC_ERROR;

    resp = i2c_read_blocking(config->port, config->i2c_address, buffer, length, false);

    if (resp == PICO_ERROR_GENERIC)
        return ISM330DLC_ERROR;

    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_i2c_write_registers(void *handle, uint8_t address, uint8_t *data, size_t length)
{
    ism330dlc_pico_i2c_config *config = (ism330dlc_pico_i2c_config *)handle;

    int resp;
    resp = i2c_write_blocking(config->port, config->i2c_address, &address, sizeof(address), true);

    if (resp == PICO_ERROR_GENERIC)
        return ISM330DLC_ERROR;

    resp = i2c_write_blocking(config->port, config->i2c_address, data, length, false);

    if (resp == PICO_ERROR_GENERIC)
        return ISM330DLC_ERROR;

    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_pico_spi4_bus_init(ism330dlc_pico_spi4_config config)
{
    (void)spi_init(config.port, config.baudrate);
    gpio_set_function(config.scl_pin, GPIO_FUNC_SPI);
    gpio_set_function(config.sdo_pin, GPIO_FUNC_SPI);
    gpio_set_function(config.sdi_pin, GPIO_FUNC_SPI);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_pico_spi4_pins_init(ism330dlc_pico_spi4_config config)
{
    gpio_init(config.cs_pin);
    gpio_set_dir(config.cs_pin, GPIO_OUT);
    gpio_put(config.cs_pin, 0);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_pico_spi4_bus_deinit(ism330dlc_pico_spi4_config config)
{
    spi_deinit(config.port);
    gpio_set_function(config.scl_pin, GPIO_FUNC_SIO);
    gpio_set_function(config.sdi_pin, GPIO_FUNC_SIO);
    gpio_set_function(config.sdo_pin, GPIO_FUNC_SIO);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_pico_spi4_pins_deinit(ism330dlc_pico_spi4_config config)
{
    gpio_deinit(config.cs_pin);

    return ISM330DLC_SUCCESS;
}

ism330dlc_status_t ism330dlc_spi4_read_registers(void *handle, uint8_t address, uint8_t *buffer, size_t length)
{
    ism330dlc_pico_spi4_config *config = (ism330dlc_pico_spi4_config *)handle;
    // The MSB is set as 1 indicating a read command
    uint8_t command = 0x80 | address;
    gpio_put(config->cs_pin, 0);

    int written = spi_write_blocking(config->port, &command, sizeof(command));
    if (written != 1)
    {
        gpio_put(config->cs_pin, 1);
        return ISM330DLC_ERROR;
    }

    int read = spi_read_blocking(config->port, 0x00, buffer, length);
    if (read != length)
    {
        gpio_put(config->cs_pin, 1);
        return ISM330DLC_ERROR;
    }

    gpio_put(config->cs_pin, 1);

    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_spi4_write_registers(void *handle, uint8_t address, uint8_t *data, size_t length)
{
    ism330dlc_pico_spi4_config *config = (ism330dlc_pico_spi4_config *)handle;
    // The MSB is set as 0 indicating a write command
    uint8_t command = 0x7F | address;
    gpio_put(config->cs_pin, 0);

    int written = spi_write_blocking(config->port, &command, sizeof(command));
    if (written != 1)
    {
        gpio_put(config->cs_pin, 1);
        return ISM330DLC_ERROR;
    }

    written = spi_write_blocking(config->port, data, length);
    if (written != length)
    {
        gpio_put(config->cs_pin, 1);
        return ISM330DLC_ERROR;
    }

    gpio_put(config->cs_pin, 1);

    return ISM330DLC_SUCCESS;
};

ism330dlc_status_t ism330dlc_spi3_read_registers(void *handle, uint8_t address, uint8_t *buffer, size_t length)
{
    assert(false && "ERROR SPI3 IS UNAVAILABLE FOR PICO");
    return ISM330DLC_ERROR;
};

ism330dlc_status_t ism330dlc_spi3_write_registers(void *handle, uint8_t address, uint8_t *data, size_t length)
{
    assert(false && "ERROR SPI3 IS UNAVAILABLE FOR PICO");
    return ISM330DLC_ERROR;
};
