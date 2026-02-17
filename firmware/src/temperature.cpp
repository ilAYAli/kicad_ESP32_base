#include "temperature.hpp"
#include "pin_config.hpp"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
static const char *TAG = "TEMP";
// ADT7310 Register addresses
enum {
    ADT7310_REG_TEMP   = 0x02,
    ADT7310_REG_ID     = 0x03,
    ADT7310_REG_CONFIG = 0x01
};
// Command generators
static inline uint8_t ADT7310_CMD_READ(uint8_t reg) {
    return 0x40 | (reg << 3);
}
static inline uint8_t ADT7310_CMD_WRITE(uint8_t reg) {
    return 0x00 | (reg << 3);
}
static const uint8_t ADT7310_ID_EXPECTED = 0xC2;
static spi_device_handle_t spi_temp;
esp_err_t temp_sensor_init(void)
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = static_cast<int>(Pin::ADT_MOSI);
    bus_cfg.miso_io_num = static_cast<int>(Pin::ADT_MISO);
    bus_cfg.sclk_io_num = static_cast<int>(Pin::ADT_CLK);
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.command_bits = 8;
    dev_cfg.clock_speed_hz = 50000;
    dev_cfg.mode = 3;
    dev_cfg.spics_io_num = static_cast<int>(Pin::ADT_CS);
    dev_cfg.queue_size = 7;
    dev_cfg.flags = SPI_DEVICE_HALFDUPLEX;
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_temp);
    if (ret != ESP_OK) return ret;
    uint8_t reset[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    spi_transaction_t t_reset = {};
    t_reset.length = 32;
    t_reset.tx_buffer = reset;
    spi_device_transmit(spi_temp, &t_reset);
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t id = 0;
    spi_transaction_t t_id = {};
    t_id.cmd = ADT7310_CMD_READ(ADT7310_REG_ID);
    t_id.rxlength = 8;
    t_id.rx_buffer = &id;
    spi_device_transmit(spi_temp, &t_id);
    if ((id & 0xFE) != ADT7310_ID_EXPECTED) {
        ESP_LOGE(TAG, "ADT7310 ID mismatch: 0x%02X (expected 0x%02X)", id, ADT7310_ID_EXPECTED);
        return ESP_ERR_NOT_FOUND;
    }
    spi_transaction_t t_cfg = {};
    t_cfg.cmd = ADT7310_CMD_WRITE(ADT7310_REG_CONFIG);
    t_cfg.length = 8;
    t_cfg.tx_data[0] = 0x80;
    t_cfg.flags = SPI_TRANS_USE_TXDATA;
    return spi_device_transmit(spi_temp, &t_cfg);
}
static float calibrate(float raw_temp)
{
    // Measured: -5.3 -> True: -5.3
    // Measured: 25.5 -> True: 22.7
    const float raw_low = -5.3f;
    const float ref_low = -5.3f;
    const float raw_high = 25.5f;
    const float ref_high = 22.7f;
    // Calculate Gain (Slope)
    const float gain = (ref_high - ref_low) / (raw_high - raw_low);
    // Apply linear correction
    return ref_low + (raw_temp - raw_low) * gain;
}
float temp_sensor_read(void)
{
    uint8_t rx[2] = {0};
    spi_transaction_t t = {};
    t.cmd = ADT7310_CMD_READ(ADT7310_REG_TEMP);
    t.rxlength = 16;
    t.rx_buffer = rx;
    if (spi_device_transmit(spi_temp, &t) != ESP_OK) {
        return -999.0f;
    }
    int16_t raw = (rx[0] << 8) | rx[1];
    return calibrate(raw / 128.0f);
}
