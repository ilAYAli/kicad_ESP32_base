#pragma once
#include "esp_err.h"
#include <cstdbool>
#ifdef __cplusplus
extern "C" {
esp_err_t webserver_start(void);
void webserver_stop(void);
void webserver_update_temp_data(float current, float min, float max);
bool webserver_is_running(void);
}
#endif
