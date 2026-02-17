#include "ota.hpp"
#include "lcd.hpp"
#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
static const char *TAG = "OTA";
static bool ota_in_progress = false;
static LCD* g_ota_lcd = nullptr;
extern const uint8_t server_cert_pem_start[] asm("_binary_server_cert_pem_start");
bool ota_is_in_progress(void)
{
    return ota_in_progress;
}
static long long get_remote_version(void)
{
    ESP_LOGI(TAG, "Fetching version from: %s", OTA_VERSION_URL);
    esp_http_client_config_t config = {};
    config.url = OTA_VERSION_URL;
    config.cert_pem = (char *)server_cert_pem_start;
    config.timeout_ms = 10000;
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return -1;
    }
    long long remote_version = -1;
    ESP_LOGI(TAG, "Opening connection...");
    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Connection opened, fetching headers...");
        int content_length = esp_http_client_fetch_headers(client);
        ESP_LOGI(TAG, "Content length: %d", content_length);
        char buffer[32];
        int len = esp_http_client_read_response(client, buffer, sizeof(buffer) - 1);
        ESP_LOGI(TAG, "Read %d bytes", len);
        if (len > 0) {
            buffer[len] = '\0';
            remote_version = atoll(buffer);
            ESP_LOGI(TAG, "Remote version: %lld, Current version: %lld", remote_version, (long long)OTA_CURRENT_VERSION);
        }
    } else {
        ESP_LOGE(TAG, "Failed to open connection: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    return remote_version;
}
static void ota_progress_callback(int progress, int total)
{
    static int last_percent = -1;
    int percent = (progress * 100) / total;
    if (percent != last_percent && percent % 10 == 0) {
        ESP_LOGI(TAG, "OTA Progress: %d%%", percent);
        if (g_ota_lcd && g_ota_lcd->isValid()) {
            g_ota_lcd->showMessage("UPDATING...", percent);
        }
        last_percent = percent;
    }
}
esp_err_t ota_check_and_update(LCD* lcd)
{
    g_ota_lcd = lcd;
    ESP_LOGI(TAG, "Checking for updates...");
    long long remote_version = get_remote_version();
    if (remote_version < 0) {
        ESP_LOGE(TAG, "Failed to get remote version");
        return ESP_FAIL;
    }
    if (remote_version <= OTA_CURRENT_VERSION) {
        ESP_LOGI(TAG, "Already up to date (remote: %lld, current: %lld)", remote_version, (long long)OTA_CURRENT_VERSION);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Update available! Starting OTA...");
    ota_in_progress = true;
    if (g_ota_lcd && g_ota_lcd->isValid()) {
        g_ota_lcd->showMessage("UPDATING...", -1);
    }
    esp_http_client_config_t http_config = {};
    http_config.url = OTA_BINARY_URL;
    http_config.cert_pem = (char *)server_cert_pem_start;
    http_config.timeout_ms = 30000;
    esp_https_ota_config_t ota_config = {};
    ota_config.http_config = &http_config;
    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_config, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        ota_in_progress = false;
        return err;
    }
    esp_app_desc_t app_desc;
    err = esp_https_ota_get_img_desc(ota_handle, &app_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get image desc: %s", esp_err_to_name(err));
        esp_https_ota_abort(ota_handle);
        ota_in_progress = false;
        return err;
    }
    ESP_LOGI(TAG, "New firmware version: %s", app_desc.version);
    while (1) {
        err = esp_https_ota_perform(ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }
        int total = esp_https_ota_get_image_size(ota_handle);
        int downloaded = esp_https_ota_get_image_len_read(ota_handle);
        ota_progress_callback(downloaded, total);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(err));
        esp_https_ota_abort(ota_handle);
        ota_in_progress = false;
        return err;
    }
    err = esp_https_ota_finish(ota_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful! Restarting...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA finish failed: %s", esp_err_to_name(err));
        ota_in_progress = false;
    }
    return err;
}
static void ota_task(void *pvParameter)
{
    LCD* lcd = static_cast<LCD*>(pvParameter);
    ESP_LOGI(TAG, "OTA task started, waiting 10 seconds...");
    vTaskDelay(pdMS_TO_TICKS(10000));
    while (1) {
        ESP_LOGI(TAG, "Starting OTA check now...");
        ota_check_and_update(lcd);
        // Check every 5 minutes (300000 ms)
        ESP_LOGI(TAG, "Next OTA check in 5 minutes...");
        vTaskDelay(pdMS_TO_TICKS(300000));
    }
    vTaskDelete(NULL);  // Never reached
}
void ota_start_background_check(LCD* lcd)
{
    xTaskCreate(&ota_task, "ota_task", 8192, lcd, 5, NULL);
}
