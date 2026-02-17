#pragma once
#include <cstdint>
#include <cstdbool>
class LCD;
#ifdef __cplusplus
extern "C" {
// Display text instructions and QR code on LCD
// line1: First line of text (e.g., "connect to wifi:")
// line2: Second line of text (e.g., "PROV_XXXXXX")
// qr_data: Data to encode in QR (e.g., "http://192.168.4.1")
bool qr_display_on_lcd(LCD* lcd, const char *line1, const char *line2, const char *qr_data);
}
#endif
