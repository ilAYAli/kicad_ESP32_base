#include "lcd.hpp"
#include "pin_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "font5x7.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
// Pin aliases for LCD
constexpr auto LCD_PIN_CS = static_cast<int>(Pin::LCD_CS);
constexpr auto LCD_PIN_DC = static_cast<int>(Pin::LCD_DC);
constexpr auto LCD_PIN_RST = static_cast<int>(Pin::LCD_RST);
constexpr auto LCD_PIN_BCKL = static_cast<int>(Pin::LCD_BCKL);
// LCD hardware configuration
enum {
    LCD_BACKLIGHT_CHANNEL = LEDC_CHANNEL_0,
    LCD_BACKLIGHT_TIMER = LEDC_TIMER_0
};
// ST7735 Commands
enum {
    ST7735_CASET  = 0x2A,
    ST7735_RASET  = 0x2B,
    ST7735_RAMWR  = 0x2C,
    ST7735_MADCTL = 0x36,
    ST7735_COLMOD = 0x3A,
    ST7735_SLPOUT = 0x11,
    ST7735_DISPON = 0x29,
    ST7735_SWRST  = 0x01
};
// Layout regions
static const int REGION_HEADER_Y = 0;
static const int REGION_HEADER_H = (LCD_HEIGHT / 8);
static const int REGION_STATUS_Y = (REGION_HEADER_Y + REGION_HEADER_H);
static const int REGION_STATUS_H = (LCD_HEIGHT / 5);
static const int REGION_BODY_Y = (REGION_STATUS_Y + REGION_STATUS_H);
static const int REGION_BODY_H = (LCD_HEIGHT / 2);
static const int REGION_FOOTER_Y = (REGION_BODY_Y + REGION_BODY_H);
// Header layout
static const int HEADER_PADDING = (LCD_WIDTH / 20);
static const int HEADER_TEXT_Y = (REGION_HEADER_Y + 8);
static const int HEADER_ICON_Y = (REGION_HEADER_Y + 8);
static const int HEADER_ICON_SPACING = (LCD_WIDTH / 10);
static const int HEADER_WIFI_X = (LCD_WIDTH - HEADER_PADDING - 13);
static const int HEADER_BT_X = (HEADER_WIFI_X - HEADER_ICON_SPACING);
// Body layout
static const int BODY_PADDING = (LCD_WIDTH / 20);
static const int BODY_GRAPH_X = BODY_PADDING;
static const int BODY_GRAPH_Y = REGION_BODY_Y;
static const int BODY_GRAPH_W = (LCD_WIDTH - 2 * BODY_PADDING);
static const int BODY_GRAPH_H = REGION_BODY_H;
// Status layout
static const int STATUS_TEXT_X = BODY_PADDING;
static const int STATUS_TEXT_Y = (REGION_STATUS_Y + 15);
static const int STATUS_VALUE_SCALE = 2;
// Footer layout
static const int FOOTER_PADDING = (LCD_WIDTH / 20);
static const int FOOTER_VALUE_X = FOOTER_PADDING;
static const int FOOTER_VALUE_Y = (REGION_FOOTER_Y + 10);
static const int FOOTER_VALUE_SCALE = 2;
static const int FOOTER_VALUE_W = (LCD_WIDTH / 4);
static const int FOOTER_MIN_LABEL_X = (LCD_WIDTH / 3);
static const int FOOTER_MIN_VALUE_X = (LCD_WIDTH / 3 + 30);
static const int FOOTER_MIN_Y = (REGION_FOOTER_Y + 10);
static const int FOOTER_MAX_LABEL_X = (LCD_WIDTH * 2 / 3 - 20);
static const int FOOTER_MAX_VALUE_X = (LCD_WIDTH * 2 / 3 + 10);
static const int FOOTER_MAX_Y = (REGION_FOOTER_Y + 10);
static spi_device_handle_t spi_lcd;
static uint16_t *line_buf = NULL;
static SemaphoreHandle_t lcd_mutex = NULL;
// Forward declarations
static void lcd_draw_string(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale);
struct lcd_graph {
    float *history;
    int capacity;
    int write_idx;
    int sample_count;
    float min_val;
    float max_val;
};
static inline void cmd(uint8_t c)
{
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 0);
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &c;
    spi_device_polling_transmit(spi_lcd, &t);
}
static inline void data(const uint8_t *d, int len)
{
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 1);
    spi_transaction_t t = {};
    t.length = static_cast<size_t>(len) * 8;
    t.tx_buffer = d;
    spi_device_polling_transmit(spi_lcd, &t);
}
void lcd_set_brightness(uint8_t brightness)
{
    if (brightness > 100) brightness = 100;
    uint32_t duty = (brightness * 255) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(LCD_BACKLIGHT_CHANNEL), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(LCD_BACKLIGHT_CHANNEL));
}
static void set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    uint8_t buf[4];
    uint16_t x_end = x + w - 1;
    uint16_t y_end = y + h - 1;
    cmd(ST7735_CASET);
    buf[0] = x >> 8; buf[1] = x & 0xFF; buf[2] = x_end >> 8; buf[3] = x_end & 0xFF;
    data(buf, 4);
    cmd(ST7735_RASET);
    buf[0] = y >> 8; buf[1] = y & 0xFF; buf[2] = y_end >> 8; buf[3] = y_end & 0xFF;
    data(buf, 4);
    cmd(ST7735_RAMWR);
}
static uint16_t lcd_blend(uint16_t c1, uint16_t c2, uint8_t alpha)
{
    uint8_t r1 = (c1 >> 11) & 0x1F, g1 = (c1 >> 5) & 0x3F, b1 = c1 & 0x1F;
    uint8_t r2 = (c2 >> 11) & 0x1F, g2 = (c2 >> 5) & 0x3F, b2 = c2 & 0x1F;
    uint8_t r = (r1 * (255 - alpha) + r2 * alpha) / 255;
    uint8_t g = (g1 * (255 - alpha) + g2 * alpha) / 255;
    uint8_t b = (b1 * (255 - alpha) + b2 * alpha) / 255;
    return (r << 11) | (g << 5) | b;
}
static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (!line_buf) {
        line_buf = static_cast<uint16_t*>(heap_caps_malloc(LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_DMA));
        if (!line_buf) return;
    }
    set_window(x, y, w, h);
    uint16_t swapped = __builtin_bswap16(color);
    for (int i = 0; i < w; i++)
        line_buf[i] = swapped;
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 1);
    spi_transaction_t t = {};
    t.length = static_cast<size_t>(w) * 16;
    t.tx_buffer = line_buf;
    for (int row = 0; row < h; row++) {
        spi_device_transmit(spi_lcd, &t);
    }
}
void lcd_fill_rect_public(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    lcd_fill_rect(x, y, w, h, color);
}
void lcd_draw_string_public(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    lcd_draw_string(x, y, s, fg, bg, scale);
}
static void lcd_fill_gradient_v(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c1, uint16_t c2)
{
    if (!line_buf) {
        line_buf = static_cast<uint16_t*>(heap_caps_malloc(LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_DMA));
        if (!line_buf) return;
    }
    set_window(x, y, w, h);
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 1);
    for (int row = 0; row < h; row++) {
        uint8_t alpha = (row * 255) / h;
        uint16_t color = lcd_blend(c1, c2, alpha);
        uint16_t swapped = __builtin_bswap16(color);
        for (int i = 0; i < w; i++) line_buf[i] = swapped;
        spi_transaction_t t = {};
        t.length = static_cast<size_t>(w) * 16;
        t.tx_buffer = line_buf;
        spi_device_transmit(spi_lcd, &t);
    }
}
static void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint8_t thickness)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (1) {
        for (int ty = -(thickness/2); ty <= thickness/2; ty++) {
            for (int tx = -(thickness/2); tx <= thickness/2; tx++) {
                lcd_fill_rect(x0 + tx, y0 + ty, 1, 1, color);
            }
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
static void lcd_draw_filled_circle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color)
{
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if (x*x + y*y <= r*r) {
                lcd_fill_rect(cx + x, cy + y, 1, 1, color);
            }
        }
    }
}
static void lcd_draw_rounded_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color)
{
    lcd_fill_rect(x + r, y, w - 2*r, 1, color);
    lcd_fill_rect(x + r, y + h - 1, w - 2*r, 1, color);
    lcd_fill_rect(x, y + r, 1, h - 2*r, color);
    lcd_fill_rect(x + w - 1, y + r, 1, h - 2*r, color);
    int xc = r, yc = 0;
    int err = 0;
    while (xc >= yc) {
        lcd_fill_rect(x + r - xc, y + r - yc, 1, 1, color);
        lcd_fill_rect(x + w - r + xc - 1, y + r - yc, 1, 1, color);
        lcd_fill_rect(x + r - xc, y + h - r + yc - 1, 1, 1, color);
        lcd_fill_rect(x + w - r + xc - 1, y + h - r + yc - 1, 1, 1, color);
        lcd_fill_rect(x + r - yc, y + r - xc, 1, 1, color);
        lcd_fill_rect(x + w - r + yc - 1, y + r - xc, 1, 1, color);
        lcd_fill_rect(x + r - yc, y + h - r + xc - 1, 1, 1, color);
        lcd_fill_rect(x + w - r + yc - 1, y + h - r + xc - 1, 1, 1, color);
        yc++;
        err += 1 + 2*yc;
        if (2*(err-xc) + 1 > 0) {
            xc--;
            err += 1 - 2*xc;
        }
    }
}
static void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
        c = '?';
    }
    const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];
    for (int col = 0; col < FONT_WIDTH; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < FONT_HEIGHT; row++) {
            uint16_t color = (line & (1 << row)) ? fg : bg;
            if (color != bg || fg != bg) {
                lcd_fill_rect(x + col * scale, y + row * scale, scale, scale, color);
            }
        }
    }
}
static void lcd_draw_string(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (!s || !lcd_mutex) return;
    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    while (*s) {
        lcd_draw_char(x, y, *s++, fg, bg, scale);
        x += (FONT_WIDTH + 1) * scale;
    }
    xSemaphoreGive(lcd_mutex);
}
void lcd_draw_bt_indicator(uint16_t x, uint16_t y, bool connected, uint8_t pulse)
{
    uint16_t color = connected ? C_BT : C_TEXT_DIM;
    int w = 18;
    int h = 26;
    int x_mid    = x + (w / 2);
    int x_right  = x + w;
    int x_left   = x;
    int y_top     = y;
    int y_bottom  = y + h;
    int y_mid     = y + (h / 2);
    int y_upper_q = y + (h / 5);
    int y_lower_q = y + (4 * h / 5);
    lcd_draw_line(x_mid, y_top, x_mid, y_bottom, color, 2);
    lcd_draw_line(x_mid, y_top, x_right, y_upper_q, color, 2);
    lcd_draw_line(x_right, y_upper_q, x_mid, y_mid, color, 2);
    lcd_draw_line(x_mid, y_mid, x_right, y_lower_q, color, 2);
    lcd_draw_line(x_right, y_lower_q, x_mid, y_bottom, color, 2);
    lcd_draw_line(x_left, y_upper_q, x_mid, y_mid, color, 2);
    lcd_draw_line(x_left, y_lower_q, x_mid, y_mid, color, 2);
}
static void lcd_draw_wifi_arc(int cx, int cy, int r, int thickness, uint16_t color)
{
    for (int i = 0; i < thickness; i++) {
        int cur_r = r + i;
        int x = cur_r;
        int y = 0;
        int err = 0;
        while (x >= y) {
            lcd_fill_rect(cx + y, cy - x, 1, 1, color);
            lcd_fill_rect(cx - y, cy - x, 1, 1, color);
            y++;
            err += 1 + 2 * y;
            if (2 * (err - x) + 1 > 0) {
                x--;
                err += 1 - 2 * x;
            }
        }
    }
}
void lcd_draw_wifi_indicator(uint16_t x, uint16_t y, bool connected)
{
    uint16_t color = connected ? C_WIFI : C_TEXT_DIM;
    int cx = x + 12;
    int cy = y + 20;
    lcd_draw_filled_circle(cx, cy, 2, color);
    lcd_draw_wifi_arc(cx, cy, 7, 2, color);
    lcd_draw_wifi_arc(cx, cy, 13, 2, color);
    lcd_draw_wifi_arc(cx, cy, 19, 2, color);
}
void lcd_draw_panel(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    lcd_fill_gradient_v(x, y, w, h, C_PANEL, C_PANEL_DK);
    lcd_draw_rounded_rect(x, y, w, h, 8, lcd_blend(C_ACCENT, C_PANEL, 200));
}
static void lcd_draw_graph_frame(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    lcd_draw_panel(x, y, w, h);
    for (int i = 1; i < 4; i++) {
        int yg = y + (h * i / 4);
        for (int xg = x + 5; xg < x + w - 5; xg += 4) {
            lcd_fill_rect(xg, yg, 2, 1, C_GRID);
        }
    }
    for (int i = 1; i < 6; i++) {
        int xg = x + (w * i / 6);
        for (int yg = y + 5; yg < y + h - 5; yg += 4) {
            lcd_fill_rect(xg, yg, 1, 2, C_GRID);
        }
    }
}
static void lcd_draw_graph_plot(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                float *history, int history_len, int hist_idx,
                                int sample_count, bool full_redraw) {
    if (sample_count < 1) return;
    float min_t = 100.0f, max_t = -100.0f;
    for (int i = 0; i < history_len; i++) {
        if (history[i] < min_t) min_t = history[i];
        if (history[i] > max_t) max_t = history[i];
    }
    float range = (max_t - min_t < 2.0f) ? 2.0f : (max_t - min_t);
    min_t -= range * 0.1f;
    max_t += range * 0.1f;
    range = max_t - min_t;
    int margin = 10;
    int plot_h = h - 2 * margin;
    int plot_w = w - 2 * margin;
    if (full_redraw) {
        lcd_draw_graph_frame(x, y, w, h);
    }
    uint16_t *row_buf = static_cast<uint16_t*>(heap_caps_malloc(plot_w * sizeof(uint16_t), MALLOC_CAP_DMA));
    if (!row_buf)
        return;
    bool *graph_points = static_cast<bool*>(malloc(plot_w * plot_h * sizeof(bool)));
    if (!graph_points) {
        free(row_buf);
        return;
    }
    memset(graph_points, 0, plot_w * plot_h * sizeof(bool));
    if (sample_count >= 2) {
        int samples_to_draw = (sample_count < plot_w) ? sample_count : plot_w;
        int buf_start = (hist_idx - samples_to_draw + history_len) % history_len;
        for (int i = 1; i < samples_to_draw; i++) {
            int idx1 = (buf_start + i - 1) % history_len;
            int idx2 = (buf_start + i) % history_len;
            int y1 = plot_h - (int)((history[idx1] - min_t) * plot_h / range);
            int y2 = plot_h - (int)((history[idx2] - min_t) * plot_h / range);
            y1 = (y1 < 0) ? 0 : (y1 > plot_h) ? plot_h : y1;
            y2 = (y2 < 0) ? 0 : (y2 > plot_h) ? plot_h : y2;
            int x0 = i - 1, x1 = i;
            int dx = 1, dy = abs(y2 - y1);
            int sy = (y1 < y2) ? 1 : -1;
            int err = dx - dy;
            int cx = x0, cy = y1;
            while (1) {
                if (cx >= 0 && cx < plot_w && cy >= 0 && cy < plot_h) {
                    graph_points[cy * plot_w + cx] = true;
                    if (cy + 1 < plot_h) graph_points[(cy + 1) * plot_w + cx] = true;
                }
                if (cx == x1 && cy == y2)
                    break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; cx += 1; }
                if (e2 < dx) { err += dx; cy += sy; }
            }
        }
    }
    set_window(x + margin, y + margin, plot_w, plot_h);
    gpio_set_level(static_cast<gpio_num_t>(LCD_PIN_DC), 1);
    uint16_t bg_color = __builtin_bswap16(C_PANEL_DK);
    uint16_t grid_color = __builtin_bswap16(C_GRID);
    uint16_t line_color = __builtin_bswap16(C_ACCENT);
    for (int row = 0; row < plot_h; row++) {
        bool is_grid_row = false;
        for (int i = 1; i < 4; i++) {
            if (row == (plot_h * i / 4)) {
                is_grid_row = true;
                break;
            }
        }
        for (int col = 0; col < plot_w; col++) {
            bool is_grid_col = false;
            for (int i = 1; i < 6; i++) {
                if (col == (plot_w * i / 6)) {
                    is_grid_col = true;
                    break;
                }
            }
            if (graph_points[row * plot_w + col]) {
                row_buf[col] = line_color;
            } else if ((is_grid_row && (col % 4 == 0 || col % 4 == 1)) ||
                       (is_grid_col && (row % 4 == 0 || row % 4 == 1))) {
                row_buf[col] = grid_color;
            } else {
                row_buf[col] = bg_color;
            }
        }
        spi_transaction_t t = {};
        t.length = static_cast<size_t>(plot_w) * 16;
        t.tx_buffer = row_buf;
        spi_device_transmit(spi_lcd, &t);
    }
    free(graph_points);
    free(row_buf);
}
static bool lcd_available = false;
esp_err_t lcd_init(void)
{
    lcd_mutex = xSemaphoreCreateMutex();
    if (!lcd_mutex) return ESP_FAIL;
    gpio_set_direction(static_cast<gpio_num_t>(Pin::LCD_DC), GPIO_MODE_OUTPUT);
    gpio_set_direction(static_cast<gpio_num_t>(Pin::LCD_RST), GPIO_MODE_OUTPUT);
    gpio_set_direction(static_cast<gpio_num_t>(Pin::LCD_BCKL), GPIO_MODE_OUTPUT);
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = 20000000;
    dev_cfg.spics_io_num = static_cast<int>(Pin::LCD_CS);
    dev_cfg.queue_size = 7;
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_lcd);
    if (ret != ESP_OK) {
        ESP_LOGW("LCD", "Failed to add SPI device - LCD may not be present");
        return ret;
    }
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_RST), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(Pin::LCD_RST), 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(ST7735_SWRST);
    vTaskDelay(pdMS_TO_TICKS(120));
    uint8_t madctl = 0xA8;
    cmd(ST7735_MADCTL);
    data(&madctl, 1);
    uint8_t colmod = 0x55;
    cmd(ST7735_COLMOD);
    data(&colmod, 1);
    cmd(ST7735_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(ST7735_DISPON);
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT;
    ledc_timer.timer_num = static_cast<ledc_timer_t>(LCD_BACKLIGHT_TIMER);
    ledc_timer.freq_hz = 5000;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = static_cast<int>(Pin::LCD_BCKL);
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = static_cast<ledc_channel_t>(LCD_BACKLIGHT_CHANNEL);
    ledc_channel.timer_sel = static_cast<ledc_timer_t>(LCD_BACKLIGHT_TIMER);
    ledc_channel.duty = 64;
    ledc_channel.hpoint = 0;
    ledc_channel_config(&ledc_channel);
    // Clear display to prevent garbage
    lcd_fill_rect_public(0, 0, LCD_WIDTH, LCD_HEIGHT, RGB565(0, 0, 0));
    lcd_available = true;
    ESP_LOGI("LCD", "LCD initialized successfully");
    return ESP_OK;
}
bool lcd_is_available(void)
{
    return lcd_available;
}
lcd_graph_t* lcd_graph_create(int history_capacity)
{
    lcd_graph_t *g = static_cast<lcd_graph_t*>(malloc(sizeof(lcd_graph_t)));
    if (!g) return NULL;
    g->history = static_cast<float*>(malloc(history_capacity * sizeof(float)));
    if (!g->history) {
        free(g);
        return NULL;
    }
    g->capacity = history_capacity;
    g->write_idx = 0;
    g->sample_count = 0;
    g->min_val = 1000.0f;
    g->max_val = -1000.0f;
    return g;
}
void lcd_graph_destroy(lcd_graph_t *graph)
{
    if (!graph) return;
    free(graph->history);
    free(graph);
}
void lcd_graph_add_sample(lcd_graph_t *graph, float value)
{
    if (!graph) return;
    if (graph->sample_count == 0) {
        for (int i = 0; i < graph->capacity; i++) {
            graph->history[i] = value;
        }
        graph->min_val = graph->max_val = value;
    }
    graph->history[graph->write_idx] = value;
    graph->write_idx = (graph->write_idx + 1) % graph->capacity;
    graph->sample_count++;
    if (value < graph->min_val) graph->min_val = value;
    if (value > graph->max_val) graph->max_val = value;
}
static void lcd_graph_draw(lcd_graph_t *graph, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if (!graph) return;
    bool range_changed = (graph->sample_count > 10 &&
                         graph->history[(graph->write_idx - 1 + graph->capacity) % graph->capacity] ==
                         (graph->min_val > graph->max_val ? graph->min_val : graph->max_val));
    lcd_draw_graph_plot(x, y, w, h, graph->history, graph->capacity,
                       graph->write_idx, graph->sample_count, range_changed);
}
void lcd_screen_init(void)
{
    lcd_fill_gradient_v(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG_TOP, C_BG_BOT);
    lcd_draw_string(FOOTER_MIN_LABEL_X, FOOTER_MIN_Y, "MIN", C_TEXT_DIM, C_BG_BOT, 1);
    lcd_draw_string(FOOTER_MAX_LABEL_X, FOOTER_MAX_Y, "MAX", C_TEXT_DIM, C_BG_BOT, 1);
}
void lcd_header_update(const char *line1, const char *line2, bool wifi_connected, bool bt_connected)
{
    static char last_line1[64] = {0};
    static char last_line2[64] = {0};
    static bool last_wifi = false;
    static bool last_bt = false;
    static uint32_t frame = 0;
    if (strcmp(line1, last_line1) != 0 || strcmp(line2, last_line2) != 0) {
        int clear_width = HEADER_BT_X - HEADER_PADDING - 5;
        lcd_fill_rect(HEADER_PADDING, HEADER_TEXT_Y, clear_width, 20, C_BG_TOP);
        lcd_draw_string(HEADER_PADDING, HEADER_TEXT_Y, line1, 
                       wifi_connected ? C_ACCENT : C_TEXT_DIM, C_BG_TOP, 1);
        lcd_draw_string(HEADER_PADDING, HEADER_TEXT_Y + 12, line2, 
                       bt_connected ? C_ACCENT : C_TEXT_DIM, C_BG_TOP, 1);
        strcpy(last_line1, line1);
        strcpy(last_line2, line2);
    }
    // Always redraw icons to ensure they appear
    if (bt_connected != last_bt || (bt_connected && frame % 20 == 0) || frame == 0) {
        uint8_t pulse = (frame % 256);
        lcd_fill_rect(HEADER_BT_X - 2, HEADER_ICON_Y, 20, 26, C_BG_TOP);
        lcd_draw_bt_indicator(HEADER_BT_X, HEADER_ICON_Y, bt_connected, pulse);
        last_bt = bt_connected;
    }
    if (wifi_connected != last_wifi || frame == 0) {
        lcd_fill_rect(HEADER_WIFI_X - 2, HEADER_ICON_Y, 26, 26, C_BG_TOP);
        lcd_draw_wifi_indicator(HEADER_WIFI_X, HEADER_ICON_Y, wifi_connected);
        last_wifi = wifi_connected;
    }
    frame++;
}
void lcd_status_update(const char *label, const char *value, uint16_t value_color)
{
    static char last_label[32] = {0};
    static char last_value[32] = {0};
    if (strcmp(label, last_label) != 0) {
        lcd_fill_rect(STATUS_TEXT_X, STATUS_TEXT_Y, 100, 16, C_BG_TOP);
        lcd_draw_string(STATUS_TEXT_X, STATUS_TEXT_Y, label, C_TEXT_DIM, C_BG_TOP, 1);
        strcpy(last_label, label);
    }
    if (strcmp(value, last_value) != 0) {
        lcd_fill_rect(STATUS_TEXT_X + 100, STATUS_TEXT_Y, 120, 16, C_BG_TOP);
        lcd_draw_string(STATUS_TEXT_X + 100, STATUS_TEXT_Y, value, value_color, C_BG_TOP, STATUS_VALUE_SCALE);
        strcpy(last_value, value);
    }
}
void lcd_body_init(void)
{
    lcd_draw_panel(BODY_GRAPH_X, BODY_GRAPH_Y, BODY_GRAPH_W, BODY_GRAPH_H);
}
void lcd_body_draw_graph(lcd_graph_t *graph)
{
    lcd_graph_draw(graph, BODY_GRAPH_X, BODY_GRAPH_Y, BODY_GRAPH_W, BODY_GRAPH_H);
}
void lcd_footer_update(float value, float min, float max, const char *unit, const char *version)
{
    static char last_value_str[16] = {0};
    static char last_min_str[16] = {0};
    static char last_max_str[16] = {0};
    static char last_version[16] = {0};
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", value);
    if (strcmp(buf, last_value_str) != 0) {
        lcd_fill_rect(FOOTER_VALUE_X, FOOTER_VALUE_Y, FOOTER_VALUE_W, 16, C_BG_BOT);
        lcd_draw_string(FOOTER_VALUE_X, FOOTER_VALUE_Y, buf, C_ACCENT, C_BG_BOT, FOOTER_VALUE_SCALE);
        strcpy(last_value_str, buf);
    }
    snprintf(buf, sizeof(buf), "%.1f%s", min, unit);
    if (strcmp(buf, last_min_str) != 0) {
        lcd_fill_rect(FOOTER_MIN_VALUE_X, FOOTER_MIN_Y, 50, 10, C_BG_BOT);
        lcd_draw_string(FOOTER_MIN_VALUE_X, FOOTER_MIN_Y, buf, C_ACCENT, C_BG_BOT, 1);
        strcpy(last_min_str, buf);
    }
    snprintf(buf, sizeof(buf), "%.1f%s", max, unit);
    if (strcmp(buf, last_max_str) != 0) {
        lcd_fill_rect(FOOTER_MAX_VALUE_X, FOOTER_MAX_Y, 60, 10, C_BG_BOT);
        lcd_draw_string(FOOTER_MAX_VALUE_X, FOOTER_MAX_Y, buf, C_WARN, C_BG_BOT, 1);
        strcpy(last_max_str, buf);
    }
    if (version && strcmp(version, last_version) != 0) {
        int version_x = LCD_WIDTH - 60;
        int version_y = REGION_FOOTER_Y + 20;
        lcd_fill_rect(version_x, version_y, 60, 10, C_BG_BOT);
        lcd_draw_string(version_x, version_y, version, C_TEXT_DIM, C_BG_BOT, 1);
        strcpy(last_version, version);
    }
}
void lcd_show_message(const char *message, int progress)
{
    static bool screen_cleared = false;
    if (!screen_cleared) {
        lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, C_BG_TOP);
        screen_cleared = true;
    }
    size_t max_chars = LCD_WIDTH / 12;
    size_t msg_len = strnlen(message, max_chars);
    int x = (LCD_WIDTH - (msg_len * 12)) / 2;
    lcd_draw_string(x, 100, message, C_ACCENT, C_BG_TOP, 2);
    if (progress >= 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d%%", progress);
        lcd_fill_rect(140, 130, 40, 20, C_BG_TOP);
        lcd_draw_string(140, 130, buf, C_GOOD, C_BG_TOP, 2);
    }
}
