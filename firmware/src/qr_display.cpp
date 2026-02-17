#include "qr_display.hpp"
#include "qrcodegen.h"
#include "lcd.hpp"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
bool qr_display_on_lcd(LCD* lcd, const char *line1, const char *line2, const char *qr_data)
{
    // Check if LCD is available
    if (!lcd || !lcd->isValid()) {
        return false;
    }
    // Allocate QR buffers on heap - too large for stack
    uint8_t *qr0 = static_cast<uint8_t*>(malloc(qrcodegen_BUFFER_LEN_MAX));
    uint8_t *tempBuffer = static_cast<uint8_t*>(malloc(qrcodegen_BUFFER_LEN_MAX));
    if (!qr0 || !tempBuffer) {
        free(qr0);
        free(tempBuffer);
        return false;
    }
    // Generate QR code
    bool ok = qrcodegen_encodeText(qr_data, tempBuffer, qr0,
                                   qrcodegen_Ecc_LOW,
                                   qrcodegen_VERSION_MIN,
                                   qrcodegen_VERSION_MAX,
                                   qrcodegen_Mask_AUTO,
                                   true);
    if (!ok) {
        free(qr0);
        free(tempBuffer);
        return false;
    }
    int size = qrcodegen_getSize(qr0);
    // Clear screen with dark background
    lcd->fillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, RGB565(15, 25, 45));
    // Draw instruction text at top - three lines
    int line1_len = strlen(line1);
    int line2_len = strlen(line2);
    const char *instruction = "Scan with phone";
    int instr_len = strlen(instruction);
    int x1 = (LCD_WIDTH - (line1_len * 6)) / 2;  // 6 pixels per char at scale 1
    int x2 = (LCD_WIDTH - (line2_len * 6)) / 2;
    int instr_x = (LCD_WIDTH - (instr_len * 6)) / 2;
    // Draw text with proper colors
    lcd->drawString(x1, 5, line1, RGB565(0, 180, 255), RGB565(15, 25, 45), 1);
    lcd->drawString(x2, 17, line2, RGB565(0, 180, 255), RGB565(15, 25, 45), 1);
    lcd->drawString(instr_x, 29, instruction, RGB565(220, 230, 240), RGB565(15, 25, 45), 1);
    // Calculate module size - leave space at top for text (45px)
    int text_area_height = 45;
    int available_height = LCD_HEIGHT - text_area_height;
    int max_display_size = (LCD_WIDTH < available_height) ? LCD_WIDTH - 40 : available_height - 40;
    int module_size = max_display_size / size;
    if (module_size < 2) module_size = 2;
    // Calculate actual QR code pixel size
    int qr_pixel_size = size * module_size;
    // Center QR horizontally, position below text
    int offset_x = (LCD_WIDTH - qr_pixel_size) / 2;
    int offset_y = text_area_height + (available_height - qr_pixel_size) / 2;
    // Add dark border around QR (quiet zone)
    int border = module_size * 2;
    lcd->fillRect(offset_x - border, offset_y - border,
                        qr_pixel_size + border * 2, qr_pixel_size + border * 2,
                        RGB565(255, 255, 255));
    // Draw QR code modules
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            if (qrcodegen_getModule(qr0, x, y)) {
                // Black module
                lcd->fillRect(offset_x + x * module_size,
                                   offset_y + y * module_size,
                                   module_size, module_size,
                                   RGB565(0, 0, 0));
            }
        }
    }
    free(qr0);
    free(tempBuffer);
    return true;
}
