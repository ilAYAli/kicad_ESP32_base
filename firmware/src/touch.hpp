#pragma once
#include "esp_err.h"
#include <cstdbool>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct TouchPoint {
    uint16_t x;
    uint16_t y;
    bool touched;
};

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t touch_init(TaskHandle_t main_task);
bool touch_read(TouchPoint* point);
bool touch_is_pressed(void);
bool touch_has_event(void);

#ifdef __cplusplus
}
#endif
