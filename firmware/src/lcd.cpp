#include "lcd.hpp"
#include "pin_config.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "font5x7.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <cmath>
// External icon files
#include "wifi_icon.c"
#include "bt_icon.c"
static const char* TAG = "LCD";
// ILI9341 Commands
namespace ILI9341 {
    constexpr uint8_t SWRST   = 0x01;
    constexpr uint8_t SLPOUT  = 0x11;
    constexpr uint8_t GAMSET  = 0x26;
    constexpr uint8_t DISPON  = 0x29;
    constexpr uint8_t CASET   = 0x2A;
    constexpr uint8_t RASET   = 0x2B;
    constexpr uint8_t RAMWR   = 0x2C;
    constexpr uint8_t MADCTL  = 0x36;
    constexpr uint8_t PIXSET  = 0x3A;
    constexpr uint8_t FRMCTR1 = 0xB1;
    constexpr uint8_t DFUNCTR = 0xB6;
    constexpr uint8_t PWCTR1  = 0xC0;
    constexpr uint8_t PWCTR2  = 0xC1;
    constexpr uint8_t VMCTR1  = 0xC5;
    constexpr uint8_t VMCTR2  = 0xC7;
    constexpr uint8_t PGAMCTRL = 0xE0;
    constexpr uint8_t NGAMCTRL = 0xE1;
}
// Hardware configuration
namespace HWConfig {
    constexpr auto LCD_BACKLIGHT_CHANNEL = LEDC_CHANNEL_0;
    constexpr auto LCD_BACKLIGHT_TIMER = LEDC_TIMER_0;
    constexpr int SPI_CLOCK_HZ = 20000000;
    constexpr int SPI_QUEUE_SIZE = 7;
}
//=============================================================================
// LCD Graph Class Implementation
//=============================================================================
LCDGraph::LCDGraph(int historyCapacity)
    : history_(std::make_unique<float[]>(historyCapacity))
    , capacity_(historyCapacity)
    , writeIdx_(0)
    , sampleCount_(0)
    , minVal_(1000.0f)
    , maxVal_(-1000.0f)
{
}
LCDGraph::~LCDGraph() = default;
LCDGraph::LCDGraph(LCDGraph&& other) noexcept
    : history_(std::move(other.history_))
    , capacity_(other.capacity_)
    , writeIdx_(other.writeIdx_)
    , sampleCount_(other.sampleCount_)
    , minVal_(other.minVal_)
    , maxVal_(other.maxVal_)
{
    other.capacity_ = 0;
    other.writeIdx_ = 0;
    other.sampleCount_ = 0;
}
LCDGraph& LCDGraph::operator=(LCDGraph&& other) noexcept {
    if (this != &other) {
        history_ = std::move(other.history_);
        capacity_ = other.capacity_;
        writeIdx_ = other.writeIdx_;
        sampleCount_ = other.sampleCount_;
        minVal_ = other.minVal_;
        maxVal_ = other.maxVal_;
        other.capacity_ = 0;
        other.writeIdx_ = 0;
        other.sampleCount_ = 0;
    }
    return *this;
}
void LCDGraph::addSample(float value) {
    if (sampleCount_ == 0) {
        // Initialize all values on first sample
        for (int i = 0; i < capacity_; i++) {
            history_[i] = value;
        }
        minVal_ = maxVal_ = value;
    }
    history_[writeIdx_] = value;
    writeIdx_ = (writeIdx_ + 1) % capacity_;
    sampleCount_++;
    minVal_ = std::min(minVal_, value);
    maxVal_ = std::max(maxVal_, value);
}
//=============================================================================
// LCD Class Implementation
//=============================================================================
LCD::LCD()
    : spiHandle_(nullptr)
    , lineBuffer_(nullptr)
    , mutex_(nullptr)
    , initialized_(false)
{
    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Failed to create LCD mutex");
        return;
    }
    // Setup GPIO pins
    gpio_set_direction(static_cast<gpio_num_t>(Pin::LCD_DC), GPIO_MODE_OUTPUT);
    gpio_set_direction(static_cast<gpio_num_t>(Pin::LCD_RST), GPIO_MODE_OUTPUT);
    gpio_set_direction(static_cast<gpio_num_t>(Pin::LCD_BCKL), GPIO_MODE_OUTPUT);
    // Add SPI device
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = HWConfig::SPI_CLOCK_HZ;
    dev_cfg.spics_io_num = static_cast<int>(Pin::LCD_CS);
    dev_cfg.queue_size = HWConfig::SPI_QUEUE_SIZE;
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spiHandle_);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add SPI device - LCD may not be present");
        return;
    }
    // Hardware reset
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_RST), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_RST), 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    // ILI9341 initialization sequence
    sendCommand(ILI9341::SWRST);
    vTaskDelay(pdMS_TO_TICKS(120));
    sendCommand(ILI9341::SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    // Power control
    sendCommand(ILI9341::PWCTR1);
    uint8_t pwr1[] = {0x23};
    sendData(pwr1, 1);
    sendCommand(ILI9341::PWCTR2);
    uint8_t pwr2[] = {0x10};
    sendData(pwr2, 1);
    // VCOM control
    sendCommand(ILI9341::VMCTR1);
    uint8_t vcom1[] = {0x3E, 0x28};
    sendData(vcom1, 2);
    sendCommand(ILI9341::VMCTR2);
    uint8_t vcom2[] = {0x86};
    sendData(vcom2, 1);
    // Memory access control (rotation/mirroring)
    sendCommand(ILI9341::MADCTL);
    uint8_t madctl = 0xE0;  // Portrait 240x320: MY=1, MX=1, MV=1, RGB=0
    sendData(&madctl, 1);
    // Pixel format: 16-bit RGB565
    sendCommand(ILI9341::PIXSET);
    uint8_t pixfmt = 0x55;
    sendData(&pixfmt, 1);
    // Frame rate control
    sendCommand(ILI9341::FRMCTR1);
    uint8_t frmctr[] = {0x00, 0x18};
    sendData(frmctr, 2);
    // Display function control
    sendCommand(ILI9341::DFUNCTR);
    uint8_t dfunc[] = {0x08, 0x82, 0x27};
    sendData(dfunc, 3);
    // Gamma set
    sendCommand(ILI9341::GAMSET);
    uint8_t gamma = 0x01;
    sendData(&gamma, 1);
    // Positive gamma correction
    sendCommand(ILI9341::PGAMCTRL);
    uint8_t pgam[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
    sendData(pgam, 15);
    // Negative gamma correction
    sendCommand(ILI9341::NGAMCTRL);
    uint8_t ngam[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
    sendData(ngam, 15);
    sendCommand(ILI9341::DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Setup PWM for backlight
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_cfg.duty_resolution = LEDC_TIMER_8_BIT;
    timer_cfg.timer_num = static_cast<ledc_timer_t>(HWConfig::LCD_BACKLIGHT_TIMER);
    timer_cfg.freq_hz = 5000;
    ledc_timer_config(&timer_cfg);
    ledc_channel_config_t ch_cfg = {};
    ch_cfg.gpio_num = static_cast<int>(Pin::LCD_BCKL);
    ch_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_cfg.channel = static_cast<ledc_channel_t>(HWConfig::LCD_BACKLIGHT_CHANNEL);
    ch_cfg.timer_sel = static_cast<ledc_timer_t>(HWConfig::LCD_BACKLIGHT_TIMER);
    ch_cfg.duty = 204;
    ledc_channel_config(&ch_cfg);
    // Clear screen
    fillRectInternal(0, 0, LCD_WIDTH, LCD_HEIGHT, 0x0000);
    initialized_ = true;
    ESP_LOGI(TAG, "LCD initialized successfully");
}
LCD::~LCD() {
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}
void LCD::sendCommand(uint8_t cmd) {
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_DC), 0);
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(spiHandle_, &t);
}
void LCD::sendData(const uint8_t* data, size_t len) {
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_DC), 1);
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spiHandle_, &t);
}
void LCD::setWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint8_t buf[4];
    uint16_t x_end = x + w - 1;
    uint16_t y_end = y + h - 1;
    sendCommand(ILI9341::CASET);
    buf[0] = x >> 8; buf[1] = x & 0xFF; buf[2] = x_end >> 8; buf[3] = x_end & 0xFF;
    sendData(buf, 4);
    sendCommand(ILI9341::RASET);
    buf[0] = y >> 8; buf[1] = y & 0xFF; buf[2] = y_end >> 8; buf[3] = y_end & 0xFF;
    sendData(buf, 4);
    sendCommand(ILI9341::RAMWR);
}
uint16_t LCD::blend(uint16_t c1, uint16_t c2, uint8_t alpha) {
    uint8_t r1 = (c1 >> 11) & 0x1F, g1 = (c1 >> 5) & 0x3F, b1 = c1 & 0x1F;
    uint8_t r2 = (c2 >> 11) & 0x1F, g2 = (c2 >> 5) & 0x3F, b2 = c2 & 0x1F;
    uint8_t r = (r1 * (255 - alpha) + r2 * alpha) / 255;
    uint8_t g = (g1 * (255 - alpha) + g2 * alpha) / 255;
    uint8_t b = (b1 * (255 - alpha) + b2 * alpha) / 255;
    return (r << 11) | (g << 5) | b;
}
void LCD::fillRectInternal(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (!lineBuffer_) {
        lineBuffer_ = std::unique_ptr<uint16_t[]>(
            static_cast<uint16_t*>(heap_caps_malloc(LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_DMA))
        );
        if (!lineBuffer_) return;
    }
    setWindow(x, y, w, h);
    uint16_t swapped = __builtin_bswap16(color);
    for (int i = 0; i < w; i++) {
        lineBuffer_[i] = swapped;
    }
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_DC), 1);
    spi_transaction_t t = {};
    t.length = static_cast<size_t>(w) * 16;
    t.tx_buffer = lineBuffer_.get();
    for (int row = 0; row < h; row++) {
        spi_device_transmit(spiHandle_, &t);
    }
}
void LCD::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    fillRectInternal(x, y, w, h, color);
    xSemaphoreGive(mutex_);
}
void LCD::fillGradientV(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c1, uint16_t c2) {
    if (!lineBuffer_) {
        lineBuffer_ = std::unique_ptr<uint16_t[]>(
            static_cast<uint16_t*>(heap_caps_malloc(LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_DMA))
        );
        if (!lineBuffer_) return;
    }
    setWindow(x, y, w, h);
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_DC), 1);
    for (int row = 0; row < h; row++) {
        uint8_t alpha = (row * 255) / h;
        uint16_t color = blend(c1, c2, alpha);
        uint16_t swapped = __builtin_bswap16(color);
        for (int i = 0; i < w; i++) {
            lineBuffer_[i] = swapped;
        }
        spi_transaction_t t = {};
        t.length = static_cast<size_t>(w) * 16;
        t.tx_buffer = lineBuffer_.get();
        spi_device_transmit(spiHandle_, &t);
    }
}
void LCD::drawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
        c = '?';
    }
    const uint8_t* glyph = font5x7[c - FONT_FIRST_CHAR];
    for (int col = 0; col < FONT_WIDTH; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < FONT_HEIGHT; row++) {
            uint16_t color = (line & (1 << row)) ? fg : bg;
            if (color != bg || fg != bg) {
                fillRectInternal(x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}
void LCD::drawString(uint16_t x, uint16_t y, const char* s, uint16_t fg, uint16_t bg, uint8_t scale) {
    if (!s || !mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    while (*s) {
        drawChar(x, y, *s++, fg, bg, scale);
        x += (FONT_WIDTH + 1) * scale;
    }
    xSemaphoreGive(mutex_);
}
void LCD::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint8_t thickness) {
    int dx = abs(static_cast<int>(x1) - static_cast<int>(x0));
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(static_cast<int>(y1) - static_cast<int>(y0));
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        for (int ty = -(thickness/2); ty <= thickness/2; ty++) {
            for (int tx = -(thickness/2); tx <= thickness/2; tx++) {
                fillRectInternal(x0 + tx, y0 + ty, 1, 1, color);
            }
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
void LCD::drawFilledCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color) {
    for (int y = -r; y <= static_cast<int>(r); y++) {
        for (int x = -r; x <= static_cast<int>(r); x++) {
            if (x*x + y*y <= static_cast<int>(r)*static_cast<int>(r)) {
                fillRectInternal(cx + x, cy + y, 1, 1, color);
            }
        }
    }
}
void LCD::drawRoundedRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
    // Draw straight edges
    fillRectInternal(x + r, y, w - 2*r, 1, color);
    fillRectInternal(x + r, y + h - 1, w - 2*r, 1, color);
    fillRectInternal(x, y + r, 1, h - 2*r, color);
    fillRectInternal(x + w - 1, y + r, 1, h - 2*r, color);
    // Draw rounded corners using Bresenham's circle algorithm
    int xc = r, yc = 0;
    int err = 0;
    while (xc >= yc) {
        fillRectInternal(x + r - xc, y + r - yc, 1, 1, color);
        fillRectInternal(x + w - r + xc - 1, y + r - yc, 1, 1, color);
        fillRectInternal(x + r - xc, y + h - r + yc - 1, 1, 1, color);
        fillRectInternal(x + w - r + xc - 1, y + h - r + yc - 1, 1, 1, color);
        fillRectInternal(x + r - yc, y + r - xc, 1, 1, color);
        fillRectInternal(x + w - r + yc - 1, y + r - xc, 1, 1, color);
        fillRectInternal(x + r - yc, y + h - r + xc - 1, 1, 1, color);
        fillRectInternal(x + w - r + yc - 1, y + h - r + xc - 1, 1, 1, color);
        yc++;
        err += 1 + 2*yc;
        if (2*(err-xc) + 1 > 0) {
            xc--;
            err += 1 - 2*xc;
        }
    }
}
void LCD::drawPanel(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    fillGradientV(x, y, w, h, LCDColors::PANEL, LCDColors::PANEL_DK);
    drawRoundedRect(x, y, w, h, 8, blend(LCDColors::ACCENT, LCDColors::PANEL, 200));
}
void LCD::drawWifiArc(int cx, int cy, int r, int thickness, uint16_t color) {
    for (int i = 0; i < thickness; i++) {
        int cur_r = r + i;
        int x = cur_r;
        int y = 0;
        int err = 0;
        while (x >= y) {
            fillRectInternal(cx + y, cy - x, 1, 1, color);
            fillRectInternal(cx - y, cy - x, 1, 1, color);
            y++;
            err += 1 + 2 * y;
            if (2 * (err - x) + 1 > 0) {
                x--;
                err += 1 - 2 * x;
            }
        }
    }
}
void LCD::drawWifiIcon(uint16_t x, uint16_t y, bool connected) {
    uint16_t color = connected ? LCDColors::WIFI : LCDColors::TEXT_DIM;
    const int w = wifi_icon.width;   // 26 pixels
    const int h = wifi_icon.height;  // 19 pixels
    const int target_h = 26;  // Match BT icon height
    // Center vertically: calculate offset for 19px icon in 26px space
    const int offset_y = (target_h - h) / 2;  // 3-4 pixels top padding
    // Draw WiFi icon centered vertically
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int idx = (row * w + col) * 3;
            unsigned char r = wifi_icon.pixel_data[idx];
            unsigned char g = wifi_icon.pixel_data[idx + 1];
            unsigned char b = wifi_icon.pixel_data[idx + 2];
            // Dark pixels (darker than light gray) are the logo
            if ((int)r + (int)g + (int)b < 600) {
                fillRectInternal(x + col, y + offset_y + row, 1, 1, color);
            }
        }
    }
}
void LCD::drawBTIcon(uint16_t x, uint16_t y, bool connected, uint8_t pulse) {
    uint16_t color = connected ? LCDColors::BT : LCDColors::TEXT_DIM;
    const int w = bt_icon.width;
    const int h = bt_icon.height;
    // Draw each pixel from RGB data (dark pixels = logo)
    for (int row = 0; row < h; row++) {
        for (int col = 0; col < w; col++) {
            int idx = (row * w + col) * 3;
            unsigned char r = bt_icon.pixel_data[idx];
            unsigned char g = bt_icon.pixel_data[idx + 1];
            unsigned char b = bt_icon.pixel_data[idx + 2];
            // Dark pixels (R+G+B < 384, i.e., average < 128) are the logo
            if ((int)r + (int)g + (int)b < 384) {
                fillRectInternal(x + col, y + row, 1, 1, color);
            }
        }
    }
}
void LCD::drawGraphFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    drawPanel(x, y, w, h);
    // Draw grid lines
    for (int i = 1; i < 4; i++) {
        int yg = y + (h * i / 4);
        fillRectInternal(x + 5, yg, w - 10, 1, LCDColors::GRID);
    }
    for (int i = 1; i < 6; i++) {
        int xg = x + (w * i / 6);
        fillRectInternal(xg, y + 5, 1, h - 10, LCDColors::GRID);
    }
}
void LCD::drawGraphPlot(LCDGraph* graph, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (!graph || graph->getSampleCount() < 1) return;
    // Calculate value range
    float min_t = 100.0f, max_t = -100.0f;
    const float* history = graph->getHistory();
    int capacity = graph->getCapacity();
    for (int i = 0; i < capacity; i++) {
        min_t = std::min(min_t, history[i]);
        max_t = std::max(max_t, history[i]);
    }
    float range = (max_t - min_t < 2.0f) ? 2.0f : (max_t - min_t);
    min_t -= range * 0.1f;
    max_t += range * 0.1f;
    range = max_t - min_t;
    int margin = 10;
    int plot_h = h - 2 * margin;
    int plot_w = w - 2 * margin;
    // Check if we need full redraw
    bool range_changed = (graph->getSampleCount() > 10 &&
                         history[(graph->getWriteIndex() - 1 + capacity) % capacity] ==
                         (graph->getMinValue() > graph->getMaxValue() ? graph->getMinValue() : graph->getMaxValue()));
    if (range_changed) {
        drawGraphFrame(x, y, w, h);
    }
    // Allocate buffers
    auto row_buf = std::unique_ptr<uint16_t[]>(
        static_cast<uint16_t*>(heap_caps_malloc(plot_w * sizeof(uint16_t), MALLOC_CAP_DMA))
    );
    if (!row_buf) return;
    auto graph_points = std::make_unique<bool[]>(plot_w * plot_h);
    std::memset(graph_points.get(), 0, plot_w * plot_h * sizeof(bool));
    // Draw line segments
    if (graph->getSampleCount() >= 2) {
        int samples_to_draw = std::min(graph->getSampleCount(), plot_w + 1);
        int buf_start = (graph->getWriteIndex() - samples_to_draw + capacity) % capacity;
        for (int i = 1; i < samples_to_draw; i++) {
            int idx1 = (buf_start + i - 1) % capacity;
            int idx2 = (buf_start + i) % capacity;
            int y1 = plot_h - static_cast<int>((history[idx1] - min_t) * plot_h / range);
            int y2 = plot_h - static_cast<int>((history[idx2] - min_t) * plot_h / range);
            y1 = std::clamp(y1, 0, plot_h - 1);
            y2 = std::clamp(y2, 0, plot_h - 1);
            // Bresenham's line algorithm
            int x0 = i - 1, x1 = i;
            int dx = 1, dy = abs(y2 - y1);
            int sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy;
            int cx = x0, cy = y1;
            while (true) {
                if (cx >= 0 && cx < plot_w && cy >= 0 && cy < plot_h) {
                    graph_points[cy * plot_w + cx] = true;
                    if (cy + 1 < plot_h) graph_points[(cy + 1) * plot_w + cx] = true;
                }
                if (cx == x1 && cy == y2) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; cx += 1; }
                if (e2 < dx) { err += dx; cy += sy; }
            }
        }
    }
    // Render to display
    setWindow(x + margin, y + margin, plot_w, plot_h);
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_DC), 1);
    uint16_t bg_color = __builtin_bswap16(LCDColors::PANEL_DK);
    uint16_t grid_color = __builtin_bswap16(LCDColors::GRID);
    uint16_t line_color = __builtin_bswap16(LCDColors::ACCENT);
    for (int row = 0; row < plot_h; row++) {
        bool is_grid_row = false;
        for (int i = 1; i < 4; i++) {
            // Grid is drawn at h * i / 4, but we render from margin offset
            int grid_pos = (h * i / 4) - margin;
            if (row == grid_pos) {
                is_grid_row = true;
                break;
            }
        }
        for (int col = 0; col < plot_w; col++) {
            bool is_grid_col = false;
            for (int i = 1; i < 6; i++) {
                // Grid is drawn at w * i / 6, but we render from margin offset
                int grid_pos = (w * i / 6) - margin;
                if (col == grid_pos) {
                    is_grid_col = true;
                    break;
                }
            }
            if (graph_points[row * plot_w + col]) {
                row_buf[col] = line_color;
            } else if (is_grid_row || is_grid_col) {
                row_buf[col] = grid_color;
            } else {
                row_buf[col] = bg_color;
            }
        }
        spi_transaction_t t = {};
        t.length = static_cast<size_t>(plot_w) * 16;
        t.tx_buffer = row_buf.get();
        spi_device_transmit(spiHandle_, &t);
    }
}
void LCD::setBrightness(uint8_t brightness) {
    if (brightness > 100) brightness = 100;
    uint32_t duty = (brightness * 255) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(HWConfig::LCD_BACKLIGHT_CHANNEL), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(HWConfig::LCD_BACKLIGHT_CHANNEL));
}
void LCD::screenInit() {
    fillGradientV(0, 0, LCD_WIDTH, LCD_HEIGHT, LCDColors::BG_TOP, LCDColors::BG_BOT);
    drawString(FOOTER_MIN_LABEL_X, FOOTER_MIN_Y, "MIN", LCDColors::TEXT_DIM, LCDColors::BG_BOT, 1);
    drawString(FOOTER_MAX_LABEL_X, FOOTER_MAX_Y, "MAX", LCDColors::TEXT_DIM, LCDColors::BG_BOT, 1);
}
void LCD::headerUpdate(const char* line1, const char* line2, bool wifiConnected, bool bleConnected) {
    static char last_line1[64] = {0};
    static char last_line2[64] = {0};
    static bool last_wifi = false;
    static bool last_bt = false;
    static uint32_t frame = 0;
    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    if (strcmp(line1, last_line1) != 0 || strcmp(line2, last_line2) != 0) {
        int clear_width = HEADER_BT_X - HEADER_PADDING - 5;
        fillRectInternal(HEADER_PADDING, HEADER_TEXT_Y, clear_width, 20, LCDColors::BG_TOP);
        int x1 = HEADER_PADDING;
        for (const char* p = line1; *p; p++) {
            drawChar(x1, HEADER_TEXT_Y, *p,
                    wifiConnected ? LCDColors::ACCENT : LCDColors::TEXT_DIM, LCDColors::BG_TOP, 1);
            x1 += 6;
        }
        int x2 = HEADER_PADDING;
        for (const char* p = line2; *p; p++) {
            drawChar(x2, HEADER_TEXT_Y + 12, *p,
                    bleConnected ? LCDColors::ACCENT : LCDColors::TEXT_DIM, LCDColors::BG_TOP, 1);
            x2 += 6;
        }
        strncpy(last_line1, line1, sizeof(last_line1) - 1);
        strncpy(last_line2, line2, sizeof(last_line2) - 1);
    }
    if (bleConnected != last_bt || (bleConnected && frame % 20 == 0) || frame == 0) {
        fillRectInternal(HEADER_BT_X - 2, HEADER_ICON_Y, 20, 26, LCDColors::BG_TOP);
        drawBTIcon(HEADER_BT_X, HEADER_ICON_Y, bleConnected, static_cast<uint8_t>(frame % 256));
        last_bt = bleConnected;
    }
    if (wifiConnected != last_wifi || frame == 0) {
        fillRectInternal(HEADER_WIFI_X - 2, HEADER_ICON_Y, 26, 26, LCDColors::BG_TOP);
        drawWifiIcon(HEADER_WIFI_X, HEADER_ICON_Y, wifiConnected);
        last_wifi = wifiConnected;
    }
    frame++;
    xSemaphoreGive(mutex_);
}
void LCD::statusUpdate(const char* label, const char* value, uint16_t valueColor) {
    static char last_label[32] = {0};
    static char last_value[32] = {0};
    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    if (strcmp(label, last_label) != 0) {
        fillRectInternal(STATUS_TEXT_X, STATUS_TEXT_Y, 100, 16, LCDColors::BG_TOP);
        int x = STATUS_TEXT_X;
        for (const char* p = label; *p; p++) {
            drawChar(x, STATUS_TEXT_Y, *p, LCDColors::TEXT_DIM, LCDColors::BG_TOP, 1);
            x += 6;
        }
        strncpy(last_label, label, sizeof(last_label) - 1);
    }
    if (strcmp(value, last_value) != 0) {
        fillRectInternal(STATUS_TEXT_X + 100, STATUS_TEXT_Y, 120, 16, LCDColors::BG_TOP);
        int x = STATUS_TEXT_X + 100;
        for (const char* p = value; *p; p++) {
            drawChar(x, STATUS_TEXT_Y, *p, valueColor, LCDColors::BG_TOP, STATUS_VALUE_SCALE);
            x += (FONT_WIDTH + 1) * STATUS_VALUE_SCALE;
        }
        strncpy(last_value, value, sizeof(last_value) - 1);
    }
    xSemaphoreGive(mutex_);
}
void LCD::bodyInit() {
    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    drawPanel(BODY_GRAPH_X, BODY_GRAPH_Y, BODY_GRAPH_W, BODY_GRAPH_H);
    xSemaphoreGive(mutex_);
}
void LCD::bodyDrawGraph(LCDGraph* graph) {
    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    if (!graph || graph->getSampleCount() < 1) {
        xSemaphoreGive(mutex_);
        return;
    }

    static bool frame_drawn = false;
    static int last_write_idx = -1;
    static float last_min_t = 1000.0f;
    static float last_max_t = -1000.0f;

    const int margin = 10;
    const int plot_w = BODY_GRAPH_W - 2 * margin;
    const int plot_h = BODY_GRAPH_H - 2 * margin;
    const int x0 = BODY_GRAPH_X + margin;
    const int y0 = BODY_GRAPH_Y + margin;

    if (!frame_drawn) {
        drawGraphFrame(BODY_GRAPH_X, BODY_GRAPH_Y, BODY_GRAPH_W, BODY_GRAPH_H);
        frame_drawn = true;
    }

    int write_idx = graph->getWriteIndex();
    if (write_idx == last_write_idx) {
        xSemaphoreGive(mutex_);
        return;
    }
    last_write_idx = write_idx;

    const float* history = graph->getHistory();
    int capacity = graph->getCapacity();
    float min_t = 100.0f;
    float max_t = -100.0f;
    for (int i = 0; i < capacity; i++) {
        min_t = std::min(min_t, history[i]);
        max_t = std::max(max_t, history[i]);
    }
    float range = (max_t - min_t < 2.0f) ? 2.0f : (max_t - min_t);
    min_t -= range * 0.1f;
    max_t += range * 0.1f;
    range = max_t - min_t;

    bool range_changed = (std::fabsf(min_t - last_min_t) > 1.0f) || (std::fabsf(max_t - last_max_t) > 1.0f);
    last_min_t = min_t;
    last_max_t = max_t;

    if (range_changed) {
        drawGraphPlot(graph, BODY_GRAPH_X, BODY_GRAPH_Y, BODY_GRAPH_W, BODY_GRAPH_H);
        xSemaphoreGive(mutex_);
        return;
    }

    int cur_idx = (write_idx - 1 + capacity) % capacity;
    int prev_idx = (write_idx - 2 + capacity) % capacity;
    float cur_val = history[cur_idx];
    float prev_val = history[prev_idx];

    int y_cur = plot_h - static_cast<int>((cur_val - min_t) * plot_h / range);
    int y_prev = plot_h - static_cast<int>((prev_val - min_t) * plot_h / range);
    y_cur = std::clamp(y_cur, 0, plot_h - 1);
    y_prev = std::clamp(y_prev, 0, plot_h - 1);

    int x_col = (write_idx - 1 + plot_w) % plot_w;
    bool is_grid_col = false;
    for (int i = 1; i < 6; i++) {
        int grid_pos = (BODY_GRAPH_W * i / 6) - margin;
        if (x_col == grid_pos) {
            is_grid_col = true;
            break;
        }
    }

    auto col_buf = std::unique_ptr<uint16_t[]>(
        static_cast<uint16_t*>(heap_caps_malloc(plot_h * sizeof(uint16_t), MALLOC_CAP_DMA))
    );
    if (!col_buf) {
        xSemaphoreGive(mutex_);
        return;
    }

    uint16_t bg_color = __builtin_bswap16(LCDColors::PANEL_DK);
    uint16_t grid_color = __builtin_bswap16(LCDColors::GRID);
    uint16_t line_color = __builtin_bswap16(LCDColors::ACCENT);
    int y_start = std::min(y_prev, y_cur);
    int y_end = std::max(y_prev, y_cur);

    for (int row = 0; row < plot_h; row++) {
        bool is_grid_row = false;
        for (int i = 1; i < 4; i++) {
            int grid_pos = (BODY_GRAPH_H * i / 4) - margin;
            if (row == grid_pos) {
                is_grid_row = true;
                break;
            }
        }
        if (row >= y_start && row <= y_end) {
            col_buf[row] = line_color;
        } else if (is_grid_row || is_grid_col) {
            col_buf[row] = grid_color;
        } else {
            col_buf[row] = bg_color;
        }
    }

    setWindow(static_cast<uint16_t>(x0 + x_col), static_cast<uint16_t>(y0), 1, static_cast<uint16_t>(plot_h));
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_DC), 1);
    spi_transaction_t t = {};
    t.length = static_cast<size_t>(plot_h) * 16;
    t.tx_buffer = col_buf.get();
    spi_device_transmit(spiHandle_, &t);
    xSemaphoreGive(mutex_);
}
void LCD::footerUpdate(float value, float min, float max, const char* unit, const char* version) {
    static char last_value_str[16] = {0};
    static char last_min_str[16] = {0};
    static char last_max_str[16] = {0};
    static char last_version[16] = {0};
    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", value);
    if (strcmp(buf, last_value_str) != 0) {
        fillRectInternal(FOOTER_VALUE_X, FOOTER_VALUE_Y, FOOTER_VALUE_W, 16, LCDColors::BG_BOT);
        int x = FOOTER_VALUE_X;
        for (const char* p = buf; *p; p++) {
            drawChar(x, FOOTER_VALUE_Y, *p, LCDColors::ACCENT, LCDColors::BG_BOT, FOOTER_VALUE_SCALE);
            x += (FONT_WIDTH + 1) * FOOTER_VALUE_SCALE;
        }
        strncpy(last_value_str, buf, sizeof(last_value_str) - 1);
    }
    snprintf(buf, sizeof(buf), "%.1f%s", min, unit);
    if (strcmp(buf, last_min_str) != 0) {
        fillRectInternal(FOOTER_MIN_VALUE_X, FOOTER_MIN_Y, 50, 10, LCDColors::BG_BOT);
        int x = FOOTER_MIN_VALUE_X;
        for (const char* p = buf; *p; p++) {
            drawChar(x, FOOTER_MIN_Y, *p, LCDColors::ACCENT, LCDColors::BG_BOT, 1);
            x += 6;
        }
        strncpy(last_min_str, buf, sizeof(last_min_str) - 1);
    }
    snprintf(buf, sizeof(buf), "%.1f%s", max, unit);
    if (strcmp(buf, last_max_str) != 0) {
        fillRectInternal(FOOTER_MAX_VALUE_X, FOOTER_MAX_Y, 60, 10, LCDColors::BG_BOT);
        int x = FOOTER_MAX_VALUE_X;
        for (const char* p = buf; *p; p++) {
            drawChar(x, FOOTER_MAX_Y, *p, LCDColors::WARN, LCDColors::BG_BOT, 1);
            x += 6;
        }
        strncpy(last_max_str, buf, sizeof(last_max_str) - 1);
    }
    if (version && strcmp(version, last_version) != 0) {
        int version_x = LCD_WIDTH - 60;
        int version_y = REGION_FOOTER_Y + 20;
        fillRectInternal(version_x, version_y, 60, 10, LCDColors::BG_BOT);
        int x = version_x;
        for (const char* p = version; *p; p++) {
            drawChar(x, version_y, *p, LCDColors::TEXT_DIM, LCDColors::BG_BOT, 1);
            x += 6;
        }
        strncpy(last_version, version, sizeof(last_version) - 1);
    }
    xSemaphoreGive(mutex_);
}
void LCD::showMessage(const char* message, int progress) {
    static bool screen_cleared = false;

    if (!mutex_) return;
    xSemaphoreTake(mutex_, portMAX_DELAY);

    // Reset screen_cleared flag when starting a new message (progress = -1)
    // This ensures the screen clears for new operations like OTA updates
    if (progress == -1) {
        screen_cleared = false;
    }

    if (!screen_cleared) {
        fillRectInternal(0, 0, LCD_WIDTH, LCD_HEIGHT, LCDColors::BG_TOP);
        screen_cleared = true;
    }
    size_t msg_len = strlen(message);
    size_t max_chars = LCD_WIDTH / 12;
    if (msg_len > max_chars) msg_len = max_chars;
    int x = (LCD_WIDTH - (msg_len * 12)) / 2;
    int char_x = x;
    for (const char* p = message; *p && (p - message) < static_cast<int>(msg_len); p++) {
        drawChar(char_x, 100, *p, LCDColors::ACCENT, LCDColors::BG_TOP, 2);
        char_x += 12;
    }
    if (progress >= 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", progress);
        fillRectInternal(140, 130, 40, 20, LCDColors::BG_TOP);
        int px = 140;
        for (const char* p = buf; *p; p++) {
            drawChar(px, 130, *p, LCDColors::GOOD, LCDColors::BG_TOP, 2);
            px += 12;
        }
    }
    xSemaphoreGive(mutex_);
}
