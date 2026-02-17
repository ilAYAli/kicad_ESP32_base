#include "touch.hpp"
#include "pin_config.hpp"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TOUCH";
static spi_device_handle_t spi_touch;
static volatile bool touch_event_pending = false;
static TaskHandle_t touch_main_task = nullptr;
static volatile uint32_t last_isr_tick = 0;  // Debounce: only wake once per 50ms

// XPT2046 Commands (with PD1-0 = 00 for IRQ-enabled mode)
#define XPT2046_CMD_X   0xD0  // Read X position (11010000)
#define XPT2046_CMD_Y   0x90  // Read Y position (10010000)

// Calibration values (adjust based on your display)
#define TOUCH_X_MIN     200
#define TOUCH_X_MAX     3900
#define TOUCH_Y_MIN     200
#define TOUCH_Y_MAX     3900

static uint16_t spi_read_touch(uint8_t command)
{
    spi_transaction_t t = {};
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 24;  // 3 bytes
    t.tx_data[0] = command;
    t.tx_data[1] = 0x00;
    t.tx_data[2] = 0x00;

    esp_err_t ret = spi_device_transmit(spi_touch, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s", esp_err_to_name(ret));
        return 0;
    }

    // XPT2046 returns 12-bit value in bits 3-14 of response
    uint16_t value = ((t.rx_data[1] << 8) | t.rx_data[2]) >> 3;
    return value & 0x0FFF;
}

static void IRAM_ATTR touch_isr_handler(void* arg)
{
    touch_event_pending = true;
    // Debounce: only wake main task once per 50ms to avoid repeated interrupts
    uint32_t current_tick = xTaskGetTickCountFromISR();
    if (current_tick - last_isr_tick >= pdMS_TO_TICKS(50) && touch_main_task) {
        last_isr_tick = current_tick;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(touch_main_task, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t touch_init(TaskHandle_t main_task)
{
    ESP_LOGI(TAG, "Initializing touch controller...");
    touch_main_task = main_task;
    // Configure IRQ pin as input with pull-up and interrupt on falling edge
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << static_cast<int>(Pin::TP_IRQ));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Trigger on falling edge (touch press)
    gpio_config(&io_conf);

    // Install ISR service (ignore error if already installed)
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(isr_ret));
        return isr_ret;
    }
    gpio_isr_handler_add(static_cast<gpio_num_t>(Pin::TP_IRQ), touch_isr_handler, nullptr);

    // Try to manually toggle CS to verify pin works
    gpio_set_direction(static_cast<gpio_num_t>(Pin::TP_CS), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(Pin::TP_CS), 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(Pin::TP_CS), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(Pin::TP_CS), 1);
    ESP_LOGI(TAG, "CS pin toggle test complete");

    // Add touch device to SPI bus (already initialized for LCD)
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = 2000000;  // 2 MHz
    dev_cfg.mode = 3;  // SPI mode 3 (CPOL=1, CPHA=1) - XPT2046 can use mode 0 or 3
    dev_cfg.spics_io_num = static_cast<int>(Pin::TP_CS);
    dev_cfg.queue_size = 1;
    dev_cfg.cs_ena_pretrans = 2;  // 2 SPI clocks CS setup
    dev_cfg.cs_ena_posttrans = 2;  // 2 SPI clocks CS hold
    // XPT2046 requires full duplex (simultaneous TX/RX), not half duplex

    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_touch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add touch device to SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Touch controller initialized - testing communication...");

    // Wait for touch controller to stabilize
    vTaskDelay(pdMS_TO_TICKS(50));

    // Try a test read to see if we get any response
    uint16_t test_x = spi_read_touch(XPT2046_CMD_X);
    uint16_t test_y = spi_read_touch(XPT2046_CMD_Y);
    ESP_LOGI(TAG, "Test read: X=%d(0x%03X) Y=%d(0x%03X)", test_x, test_x, test_y, test_y);

    if (test_x == 0 && test_y == 4095) {
        ESP_LOGW(TAG, "Touch controller not responding (may not be present or powered)");
    }

    return ESP_OK;
}

bool touch_is_pressed(void)
{
    // XPT2046 IRQ is active low when touched
    int level = gpio_get_level(static_cast<gpio_num_t>(Pin::TP_IRQ));
    return level == 0;
}

bool touch_has_event(void)
{
    return touch_event_pending;
}

bool touch_read(TouchPoint* point)
{
    if (!point) {
        return false;
    }

    touch_event_pending = false;  // Clear event flag

    // Check if touch is still pressed (debounce)
    if (!touch_is_pressed()) {
        point->touched = false;
        return false;
    }

    // Read touch coordinates
    uint16_t raw_x = spi_read_touch(XPT2046_CMD_X);
    uint16_t raw_y = spi_read_touch(XPT2046_CMD_Y);

    // Detect touch by coordinate stability instead of pressure
    // When not touched: X and Y will be unstable (jumping around)
    // When touched: X and Y will be stable and in valid range
    bool coords_valid = (raw_x > 100 && raw_x < 4000 && raw_y > 100 && raw_y < 4000);

    if (!coords_valid) {
        point->touched = false;
        return false;
    }

    // Map raw coordinates to screen coordinates (320x240)
    // Adjust calibration based on actual observed values
    if (raw_x < TOUCH_X_MIN) raw_x = TOUCH_X_MIN;
    if (raw_x > TOUCH_X_MAX) raw_x = TOUCH_X_MAX;
    if (raw_y < TOUCH_Y_MIN) raw_y = TOUCH_Y_MIN;
    if (raw_y > TOUCH_Y_MAX) raw_y = TOUCH_Y_MAX;

    // Map touch axes: screen X from touch Y, screen Y from touch X
    int16_t screen_x = ((raw_y - TOUCH_Y_MIN) * 320) / (TOUCH_Y_MAX - TOUCH_Y_MIN);
    int16_t screen_y = ((raw_x - TOUCH_X_MIN) * 240) / (TOUCH_X_MAX - TOUCH_X_MIN);
    // Apply calibration offset
    screen_x -= 10;
    // Clamp to screen bounds
    point->x = (screen_x < 0) ? 0 : (screen_x >= 320) ? 319 : screen_x;
    point->y = (screen_y < 0) ? 0 : (screen_y >= 240) ? 239 : screen_y;
    point->touched = true;
    return true;
}
