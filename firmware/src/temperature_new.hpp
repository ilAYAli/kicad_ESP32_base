#pragma once
#include "esp_err.h"
class Temperature {
public:
    static Temperature& instance();
    esp_err_t init();
    float read();
private:
    Temperature() = default;
    ~Temperature() = default;
    Temperature(const Temperature&) = delete;
    Temperature& operator=(const Temperature&) = delete;
    void* spi_device = nullptr;
};
