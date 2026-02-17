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
    // Debounce: only wake main task once per 30ms to catch quick taps
    uint32_t current_tick = xTaskGetTickCountFromISR();
    if (current_tick - last_isr_tick >= pdMS_TO_TICKS(30) && touch_main_task) {
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

    // Read multiple samples and use median for stability (rejects outliers better than mean)
    constexpr int NUM_SAMPLES = 9;  // More samples for better median
    uint16_t samples_x[NUM_SAMPLES];
    uint16_t samples_y[NUM_SAMPLES];
    int valid_samples = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t raw_x = spi_read_touch(XPT2046_CMD_X);
        uint16_t raw_y = spi_read_touch(XPT2046_CMD_Y);
        
        // Only use samples with valid coordinates
        if (raw_x > 100 && raw_x < 4000 && raw_y > 100 && raw_y < 4000) {
            samples_x[valid_samples] = raw_x;
            samples_y[valid_samples] = raw_y;
            valid_samples++;
        }
    }

    // Need at least 5 valid samples to consider touch valid
    if (valid_samples < 5) {
        point->touched = false;
        return false;
    }

    // Sort samples to find median (simple bubble sort for small array)
    for (int i = 0; i < valid_samples - 1; i++) {
        for (int j = 0; j < valid_samples - i - 1; j++) {
            if (samples_x[j] > samples_x[j + 1]) {
                uint16_t temp = samples_x[j];
                samples_x[j] = samples_x[j + 1];
                samples_x[j + 1] = temp;
            }
            if (samples_y[j] > samples_y[j + 1]) {
                uint16_t temp = samples_y[j];
                samples_y[j] = samples_y[j + 1];
                samples_y[j + 1] = temp;
            }
        }
    }

    // Use median value (middle element)
    uint16_t median_x = samples_x[valid_samples / 2];
    uint16_t median_y = samples_y[valid_samples / 2];

    // Calculate interquartile range (IQR) to detect outliers
    // This is more robust than just looking at middle 3 samples
    int q1_idx = valid_samples / 4;
    int q3_idx = (3 * valid_samples) / 4;
    
    uint16_t iqr_x = samples_x[q3_idx] - samples_x[q1_idx];
    uint16_t iqr_y = samples_y[q3_idx] - samples_y[q1_idx];

    // Also check full range to catch widely scattered samples
    uint16_t range_x = samples_x[valid_samples - 1] - samples_x[0];
    uint16_t range_y = samples_y[valid_samples - 1] - samples_y[0];

    // Reject only if spread is extremely high (very loose threshold due to noisy hardware)
    // Rely more on application-level filtering for duplicate detection
    if (iqr_x > 120 || iqr_y > 120 || range_x > 500 || range_y > 500) {
        ESP_LOGW(TAG, "Touch rejected - high spread: iqr_x=%d iqr_y=%d, range_x=%d range_y=%d, median: X=%d Y=%d", 
                 iqr_x, iqr_y, range_x, range_y, median_x, median_y);
        point->touched = false;
        return false;
    }

    uint16_t raw_x = median_x;
    uint16_t raw_y = median_y;

    // Reject phantom touches that occur immediately after a real touch with vastly different coordinates
    static uint16_t last_raw_x = 0;
    static uint16_t last_raw_y = 0;
    static uint32_t last_read_time = 0;
    uint32_t current_time = xTaskGetTickCount();
    
    if (last_raw_x != 0 && last_raw_y != 0) {
        uint32_t time_diff_ms = (current_time - last_read_time) * portTICK_PERIOD_MS;
        if (time_diff_ms < 200) {  // Within 200ms of last touch
            int raw_dx = (int)raw_x - (int)last_raw_x;
            int raw_dy = (int)raw_y - (int)last_raw_y;
            if (raw_dx < 0) raw_dx = -raw_dx;
            if (raw_dy < 0) raw_dy = -raw_dy;
            // If raw coordinates differ by more than 800 units, likely a phantom
            if (raw_dx > 800 || raw_dy > 800) {
                ESP_LOGW(TAG, "Touch rejected - phantom (too different from previous): X=%d Y=%d, prev: X=%d Y=%d, dt=%lums", 
                         raw_x, raw_y, last_raw_x, last_raw_y, time_diff_ms);
                point->touched = false;
                return false;
            }
        }
    }
    
    last_raw_x = raw_x;
    last_raw_y = raw_y;
    last_read_time = current_time;

    ESP_LOGI(TAG, "Raw touch: X=%d Y=%d (median from %d samples, iqr_x=%d iqr_y=%d)", raw_x, raw_y, valid_samples, iqr_x, iqr_y);

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
