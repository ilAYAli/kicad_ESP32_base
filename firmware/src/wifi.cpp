#include "wifi.hpp"
#include "wifi_prov.hpp"
#include "webserver.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdio.h>
static const char *TAG = "WIFI";
static bool wifi_connected = false;
static char ip_address[16] = "0.0.0.0";
static int8_t current_rssi = -100;
static uint32_t connection_start_time = 0;
static int retry_count = 0;
static const int MAX_RETRY_COUNT = 10;  // 10 retries * ~10 seconds each = ~100 seconds total
static const uint32_t RETRY_TIMEOUT_MS = 10000;  // 10 seconds per retry cycle
static void update_rssi(void)
{
    if (wifi_connected) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            current_rssi = ap_info.rssi;
        }
    } else {
        current_rssi = -100;
    }
}
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        connection_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        retry_count = 0;
        ESP_LOGI(TAG, "WiFi starting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconn = (wifi_event_sta_disconnected_t*) event_data;
        wifi_connected = false;
        current_rssi = -100;
        strcpy(ip_address, "0.0.0.0");
        webserver_stop();

        ESP_LOGW(TAG, "Disconnected from SSID:%s, reason:%d", disconn->ssid, disconn->reason);

        // Always try to reconnect after a brief delay
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconnecting...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Connected! IP: %s", ip_address);
        wifi_connected = true;
        retry_count = 0;  // Reset retry count on success
        update_rssi();

        // Use light sleep power save mode instead of modem sleep for better stability
        // WIFI_PS_MIN_MODEM can cause disconnections on some routers
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);  // More stable than MIN_MODEM
        ESP_LOGI(TAG, "WiFi power save mode enabled (MAX_MODEM)");

        webserver_start();
    }
}
// Main WiFi init - uses provisioning if not configured
esp_err_t wifi_init(LCD* lcd)
{
    // Just start provisioning - it will check if already provisioned internally
    return wifi_prov_start(lcd);
}
bool wifi_is_connected(void)
{
    return wifi_connected;
}
const char* wifi_get_ip_address(void)
{
    return ip_address;
}
int8_t wifi_get_rssi(void)
{
    update_rssi();
    return current_rssi;
}
