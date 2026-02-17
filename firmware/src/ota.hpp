#pragma once
#include "esp_err.h"
#include <cstdbool>
#define OTA_VERSION_URL "https://wahlman.no/firmware/esp32base/version.txt"
#define OTA_BINARY_URL  "https://wahlman.no/firmware/esp32base/firmware.bin"
#define OTA_CURRENT_VERSION 2602171205LL
class LCD;
#ifdef __cplusplus
extern "C" {
esp_err_t ota_check_and_update(LCD* lcd);
void ota_start_background_check(LCD* lcd);
bool ota_is_in_progress(void);
}
#endif
