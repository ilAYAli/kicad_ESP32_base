#pragma once
#include "esp_err.h"
#include <cstdbool>
#include <cstdint>
class LCD;
#ifdef __cplusplus
extern "C" {
// Main WiFi init - automatically handles provisioning if needed
esp_err_t wifi_init(LCD* lcd);
bool wifi_is_connected(void);
const char* wifi_get_ip_address(void);
int8_t wifi_get_rssi(void);
}
#endif
