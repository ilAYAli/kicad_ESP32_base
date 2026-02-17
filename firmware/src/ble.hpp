#pragma once
#include "esp_err.h"
#include <cstdbool>
#ifdef __cplusplus
extern "C" {
esp_err_t bt_init(void);
void bt_update_temp(float temp);
bool bt_is_connected(void);
const char* bt_get_mac_address(void);
const char* bt_get_id(void);
int8_t bt_get_rssi(void);
bool bt_should_shutdown(void);
void bt_shutdown(void);
}
#endif
