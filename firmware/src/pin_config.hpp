#pragma once
#include <cstdint>
// ESP32 Pin Assignments
enum class Pin : uint8_t {
    // SPI Shared Bus (SPI2)
    ADT_CLK     = 0,   // Pin 6 (GPIO0)
    ADT_MOSI    = 1,   // Pin 7 (GPIO1)
    ADT_MISO    = 8,   // Pin 10 (GPIO8)
    // ADT7310 Temperature Sensor
    ADT_CS      = 23,  // Pin 21 (GPIO23)
    // LCD (HS280S010B)
    LCD_CS      = 25,  // Pin 26 (GPIO25)
    LCD_RST     = 11,  // Pin 25 (GPIO11/TX0)
    LCD_DC      = 10,  // Pin 12 (GPIO10)
    LCD_BCKL    = 26,  // Pin 27 (GPIO26)
    // Touchpad
    TP_CS       = 12,  // Pin 12 (U0RXD, GPIO12)
    TP_IRQ      = 24,  // Pin 23 (GPIO24)
    // Input devices
    JOYSTICK    = 3,   // Pin 5 (IO3/ADC1_CH2)
    BOOT        = 28   // Pin 15 (GPIO28)
};
