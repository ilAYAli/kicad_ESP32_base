#pragma once
#include <cstdint>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
// Initializes SPI bus and ADT7310 sensor
esp_err_t temp_sensor_init(void);
// Reads temperature and returns float
float temp_sensor_read(void);
#ifdef __cplusplus
}
#endif
