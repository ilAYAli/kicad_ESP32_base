#pragma once
#include <cstdint>
#include <memory>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "freertos/semphr.h"
// RGB565 color conversion macro
#define RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
// LCD display dimensions
constexpr uint16_t LCD_WIDTH = 320;
constexpr uint16_t LCD_HEIGHT = 240;
// Color palette - C++ namespace (preferred for new code)
namespace LCDColors {
    constexpr uint16_t BG_TOP = RGB565(15, 25, 45);
    constexpr uint16_t BG_BOT = RGB565(5, 10, 20);
    constexpr uint16_t PANEL = RGB565(25, 35, 55);
    constexpr uint16_t PANEL_DK = RGB565(15, 20, 35);
    constexpr uint16_t ACCENT = RGB565(0, 180, 255);
    constexpr uint16_t GLOW = RGB565(80, 200, 255);
    constexpr uint16_t TEXT = RGB565(220, 230, 240);
    constexpr uint16_t TEXT_DIM = RGB565(140, 150, 160);
    constexpr uint16_t GRID = RGB565(40, 50, 70);
    constexpr uint16_t WARN = RGB565(255, 140, 0);
    constexpr uint16_t GOOD = RGB565(0, 255, 120);
    constexpr uint16_t BT = RGB565(100, 150, 255);  // Blue for Bluetooth
    constexpr uint16_t WIFI = RGB565(100, 200, 100);
}
// Backward compatibility macros
#define C_BG_TOP    LCDColors::BG_TOP
#define C_BG_BOT    LCDColors::BG_BOT
#define C_PANEL     LCDColors::PANEL
#define C_PANEL_DK  LCDColors::PANEL_DK
#define C_ACCENT    LCDColors::ACCENT
#define C_GLOW      LCDColors::GLOW
#define C_TEXT      LCDColors::TEXT
#define C_TEXT_DIM  LCDColors::TEXT_DIM
#define C_GRID      LCDColors::GRID
#define C_WARN      LCDColors::WARN
#define C_GOOD      LCDColors::GOOD
#define C_BT        LCDColors::BT
#define C_WIFI      LCDColors::WIFI
// Forward declaration
class LCDGraph;
// LCD display controller class - Singleton pattern
class LCD {
public:
    LCD();
    ~LCD();
    // Delete copy/move - one LCD per object
    LCD(const LCD&) = delete;
    LCD& operator=(const LCD&) = delete;
    LCD(LCD&&) = delete;
    LCD& operator=(LCD&&) = delete;
    // Check if initialization succeeded
    bool isValid() const { return initialized_; }
    void setBrightness(uint8_t brightness);
    void screenInit();
    // Section APIs - match UI structure
    void headerUpdate(const char* line1, const char* line2, bool wifiConnected, bool bleConnected);
    void statusUpdate(const char* label, const char* value, uint16_t valueColor);
    void bodyInit();
    void bodyDrawGraph(LCDGraph* graph);
    void footerUpdate(float value, float min, float max, const char* unit, const char* version);
    // Utility
    void showMessage(const char* message, int progress = -1);
    // Low-level drawing (public for QR code display)
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void drawFilledCircle(uint16_t cx, uint16_t cy, uint16_t r, uint16_t color);
    void drawString(uint16_t x, uint16_t y, const char* s, uint16_t fg, uint16_t bg, uint8_t scale = 1);
private:
    // Hardware communication
    void sendCommand(uint8_t cmd);
    void sendData(const uint8_t* data, size_t len);
    void setWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    // Drawing primitives
    void fillRectInternal(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void fillGradientV(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c1, uint16_t c2);
    void drawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);
    void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, uint8_t thickness = 1);
    void drawRoundedRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color);
    void drawPanel(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    // Icon drawing
    void drawWifiIcon(uint16_t x, uint16_t y, bool connected);
    void drawBTIcon(uint16_t x, uint16_t y, bool connected, uint8_t pulse);
    void drawWifiArc(int cx, int cy, int r, int thickness, uint16_t color);
    // Graph rendering
    void drawGraphFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    void drawGraphPlot(LCDGraph* graph, uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    // Utility
    static uint16_t blend(uint16_t c1, uint16_t c2, uint8_t alpha);
    // Member variables
    spi_device_handle_t spiHandle_;
    std::unique_ptr<uint16_t[]> lineBuffer_;
    SemaphoreHandle_t mutex_;
    bool initialized_;
    // Layout constants
    static constexpr int REGION_HEADER_Y = 0;
    static constexpr int REGION_HEADER_H = LCD_HEIGHT / 8;
    static constexpr int REGION_STATUS_Y = REGION_HEADER_Y + REGION_HEADER_H;
    static constexpr int REGION_STATUS_H = LCD_HEIGHT / 5;
    static constexpr int REGION_BODY_Y = REGION_STATUS_Y + REGION_STATUS_H;
    static constexpr int REGION_BODY_H = LCD_HEIGHT / 2;
    static constexpr int REGION_FOOTER_Y = REGION_BODY_Y + REGION_BODY_H;
    static constexpr int HEADER_PADDING = LCD_WIDTH / 20;
    static constexpr int HEADER_TEXT_Y = REGION_HEADER_Y + 8;
    static constexpr int HEADER_ICON_Y = REGION_HEADER_Y + 8;
    static constexpr int HEADER_ICON_SPACING = LCD_WIDTH / 10;
    static constexpr int HEADER_WIFI_X = LCD_WIDTH - HEADER_PADDING - 13;
    static constexpr int HEADER_BT_X = HEADER_WIFI_X - HEADER_ICON_SPACING;
    static constexpr int BODY_PADDING = LCD_WIDTH / 20;
    static constexpr int BODY_GRAPH_X = BODY_PADDING;
    static constexpr int BODY_GRAPH_Y = REGION_BODY_Y;
    static constexpr int BODY_GRAPH_W = LCD_WIDTH - 2 * BODY_PADDING;
    static constexpr int BODY_GRAPH_H = REGION_BODY_H;
    static constexpr int STATUS_TEXT_X = BODY_PADDING;
    static constexpr int STATUS_TEXT_Y = REGION_STATUS_Y + 15;
    static constexpr int STATUS_VALUE_SCALE = 2;
    static constexpr int FOOTER_PADDING = LCD_WIDTH / 20;
    static constexpr int FOOTER_VALUE_X = FOOTER_PADDING;
    static constexpr int FOOTER_VALUE_Y = REGION_FOOTER_Y + 10;
    static constexpr int FOOTER_VALUE_SCALE = 2;
    static constexpr int FOOTER_VALUE_W = LCD_WIDTH / 4;
    static constexpr int FOOTER_MIN_LABEL_X = LCD_WIDTH / 3;
    static constexpr int FOOTER_MIN_VALUE_X = LCD_WIDTH / 3 + 30;
    static constexpr int FOOTER_MIN_Y = REGION_FOOTER_Y + 10;
    static constexpr int FOOTER_MAX_LABEL_X = LCD_WIDTH * 2 / 3 - 20;
    static constexpr int FOOTER_MAX_VALUE_X = LCD_WIDTH * 2 / 3 + 10;
    static constexpr int FOOTER_MAX_Y = REGION_FOOTER_Y + 10;
};
// Graph class with RAII
class LCDGraph {
public:
    explicit LCDGraph(int historyCapacity);
    ~LCDGraph();
    // Delete copy but allow move
    LCDGraph(const LCDGraph&) = delete;
    LCDGraph& operator=(const LCDGraph&) = delete;
    LCDGraph(LCDGraph&& other) noexcept;
    LCDGraph& operator=(LCDGraph&& other) noexcept;
    void addSample(float value);
    // Accessors (for LCD class internal use)
    float* getHistory() { return history_.get(); }
    const float* getHistory() const { return history_.get(); }
    int getCapacity() const { return capacity_; }
    int getWriteIndex() const { return writeIdx_; }
    int getSampleCount() const { return sampleCount_; }
    float getMinValue() const { return minVal_; }
    float getMaxValue() const { return maxVal_; }
private:
    std::unique_ptr<float[]> history_;
    int capacity_;
    int writeIdx_;
    int sampleCount_;
    float minVal_;
    float maxVal_;
    friend class LCD;
};
