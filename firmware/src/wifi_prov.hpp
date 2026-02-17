#pragma once
#include "esp_err.h"
#include <cstdbool>
class LCD;
#ifdef __cplusplus
extern "C" {
// Start WiFi provisioning over SoftAP
// Checks if already provisioned and connects if so
// Otherwise creates AP "PROV_<MAC>" and displays QR code
esp_err_t wifi_prov_start(LCD* lcd);
// Manually start provisioning AP mode (triggered by button)
esp_err_t wifi_prov_start_ap_mode(LCD* lcd);
// Check if WiFi credentials exist in NVS
bool wifi_prov_has_credentials(void);
// Reset WiFi credentials (for testing/factory reset)
void wifi_prov_reset(void);
}
#endif
