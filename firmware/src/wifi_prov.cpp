#include "wifi_prov.hpp"
#include "wifi.hpp"
#include "qr_display.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
static const char *TAG = "WIFI_PROV";
#define NVS_NAMESPACE "wifi_config"
extern void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static const char *html_page =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<style>"
"body{font-family:Arial;max-width:400px;margin:50px auto;padding:20px;background:#f0f0f0}"
"h2{color:#333}"
"input,button{width:100%;padding:12px;margin:8px 0;box-sizing:border-box;border:1px solid #ddd;border-radius:4px}"
"button{background:#007bff;color:white;border:none;cursor:pointer;font-size:16px}"
"button:hover{background:#0056b3}"
"</style></head>"
"<body><h2>WiFi Setup</h2>"
"<form action='/save' method='POST'>"
"<input type='text' name='ssid' placeholder='WiFi Network Name' required>"
"<input type='password' name='pass' placeholder='Password (leave empty if none)'>"
"<button type='submit'>Connect</button>"
"</form></body></html>";
static const char *success_page =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<style>body{font-family:Arial;max-width:400px;margin:50px auto;padding:20px;text-align:center}</style>"
"</head><body><h2>Success!</h2><p>Device will restart and connect to your WiFi network.</p></body></html>";
static bool check_wifi_credentials_exist(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return false;
    size_t ssid_len = 0;
    err = nvs_get_str(nvs_handle, "ssid", NULL, &ssid_len);
    nvs_close(nvs_handle);
    return (err == ESP_OK && ssid_len > 0);
}
static esp_err_t connect_with_saved_credentials(void)
{
    ESP_LOGI(TAG, "Connecting with saved credentials");
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return err;
    char ssid[33] = {0};
    char password[65] = {0};
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);
    nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len);
    nvs_get_str(nvs_handle, "password", password, &pass_len);
    nvs_close(nvs_handle);
    static bool netif_init = false;
    if (!netif_init) {
        esp_netif_init();
        netif_init = true;
    }
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) return err;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);
    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);
    // Configure connection behavior for better reliability
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;  // Scan all channels instead of fast scan
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;  // Connect to strongest signal
    wifi_config.sta.threshold.rssi = -127;  // Don't filter by RSSI (accept weak signals)
    wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;  // Accept any auth mode
    wifi_config.sta.failure_retry_cnt = 5;  // Retry 5 times before giving up on this attempt
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    // ESP32-C5 supports 802.11ax (WiFi 6) which provides better range than older standards
    // Use the newer esp_wifi_set_protocols API for band-specific protocol configuration
    wifi_protocols_t protocols;
    protocols.ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX;
    protocols.ghz_5g = 0;  // Not using 5GHz
    esp_err_t proto_err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols);
    if (proto_err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi protocols enabled: 802.11b/g/n/ax (WiFi 6)");
    } else {
        ESP_LOGW(TAG, "Failed to set WiFi protocols: %s", esp_err_to_name(proto_err));
    }
    // Wait briefly for WiFi to initialize
    vTaskDelay(pdMS_TO_TICKS(100));
    // Stop auto-connect so we can scan first
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    // Perform a scan to show what APs are visible (diagnostic)
    ESP_LOGI(TAG, "Scanning for WiFi networks (ESP32 only supports 2.4GHz)...");
    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true;
    scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config.scan_time.active.min = 120;
    scan_config.scan_time.active.max = 150;
    if (esp_wifi_scan_start(&scan_config, true) == ESP_OK) {
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count > 0) {
            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
            if (ap_list && esp_wifi_scan_get_ap_records(&ap_count, ap_list) == ESP_OK) {
                ESP_LOGI(TAG, "Found %d APs:", ap_count);
                bool found_target = false;
                char correct_ssid[33] = {0};
                for (int i = 0; i < ap_count && i < 10; i++) {  // Show first 10
                    ESP_LOGI(TAG, "  [%d] SSID:'%s' Ch:%d RSSI:%d", 
                             i, ap_list[i].ssid, ap_list[i].primary, ap_list[i].rssi);
                }
                // Check all APs (not just first 10) for case-insensitive match
                for (int i = 0; i < ap_count; i++) {
                    if (strcasecmp((char*)ap_list[i].ssid, ssid) == 0) {
                        found_target = true;
                        strncpy(correct_ssid, (char*)ap_list[i].ssid, sizeof(correct_ssid) - 1);
                        break;
                    }
                }
                if (found_target) {
                    if (strcmp(correct_ssid, ssid) != 0) {
                        ESP_LOGI(TAG, "Target AP found with case mismatch: '%s' -> '%s'", ssid, correct_ssid);
                        // Update wifi_config with correct case
                        strcpy((char*)wifi_config.sta.ssid, correct_ssid);
                        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
                        ESP_LOGI(TAG, "Updated SSID to correct case: '%s'", correct_ssid);
                    } else {
                        ESP_LOGI(TAG, "Target AP '%s' found in scan!", ssid);
                    }
                } else {
                    ESP_LOGW(TAG, "Target AP '%s' NOT found! Check if it's 5GHz-only or SSID is correct", ssid);
                }
            }
            free(ap_list);
        } else {
            ESP_LOGW(TAG, "No APs found in scan!");
        }
    }
    // Start connection attempt
    ESP_LOGI(TAG, "Starting connection to '%s'...", wifi_config.sta.ssid);
    esp_wifi_connect();
    return ESP_OK;
}
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
// URL decode helper function
static void url_decode(char *dst, const char *src)
{
    char a, b;
    while (*src) {
        if (*src == '%' && ((a = src[1]) && (b = src[2])) && (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a') a -= 'a'-'A';
            if (a >= 'A') a -= ('A' - 10);
            else a -= '0';
            if (b >= 'a') b -= 'a'-'A';
            if (b >= 'A') b -= ('A' - 10);
            else b -= '0';
            *dst++ = 16*a+b;
            src+=3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}
static esp_err_t save_handler(httpd_req_t *req)
{
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    ESP_LOGI(TAG, "Received data: %s", buf);
    char ssid[33] = {0};
    char password[65] = {0};
    char ssid_encoded[100] = {0};
    char pass_encoded[100] = {0};
    // Parse: ssid=XXX&pass=YYY
    char *s = strstr(buf, "ssid=");
    char *p = strstr(buf, "&pass=");
    if (s) {
        s += 5;  // Skip "ssid="
        char *e = strchr(s, '&');
        int len = e ? (e - s) : strlen(s);
        if (len > 0 && len < 100) {
            strncpy(ssid_encoded, s, len);
            ssid_encoded[len] = '\0';
            url_decode(ssid, ssid_encoded);
        }
    }
    if (p) {
        p += 6;  // Skip "&pass="
        strncpy(pass_encoded, p, 99);
        url_decode(password, pass_encoded);
    }
    if (strlen(ssid) == 0) {
        ESP_LOGE(TAG, "No SSID provided!");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Saving WiFi credentials:");
    ESP_LOGI(TAG, "  SSID: '%s' (length: %d)", ssid, strlen(ssid));
    ESP_LOGI(TAG, "  Password: '%s' (length: %d)", password, strlen(password));
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs_handle, "ssid", ssid);
        ESP_LOGI(TAG, "SSID save result: %s", esp_err_to_name(err));
        err = nvs_set_str(nvs_handle, "password", password);
        ESP_LOGI(TAG, "Password save result: %s", esp_err_to_name(err));
        err = nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "NVS commit result: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    }
    httpd_resp_send(req, success_page, HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}
esp_err_t wifi_prov_start(LCD* lcd)
{
    (void)lcd;  // Not used in this function
    if (!check_wifi_credentials_exist()) {
        ESP_LOGI(TAG, "No WiFi credentials - skipping WiFi initialization");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "WiFi credentials found, connecting in background...");
    return connect_with_saved_credentials();
}
bool wifi_prov_has_credentials(void)
{
    return check_wifi_credentials_exist();
}
esp_err_t wifi_prov_start_ap_mode(LCD* lcd)
{
    ESP_LOGI(TAG, "Starting provisioning AP mode");
    static bool netif_init = false;
    if (!netif_init) {
        esp_netif_init();
        netif_init = true;
    }
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "pwa_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    wifi_config_t wifi_config = {};
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    strcpy((char*)wifi_config.ap.ssid, ap_ssid);
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "AP started: %s", ap_ssid);
    ESP_LOGI(TAG, "IP: 192.168.4.1");
    // Start web server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {};
        root_uri.uri = "/";
        root_uri.method = HTTP_GET;
        root_uri.handler = root_handler;
        httpd_register_uri_handler(server, &root_uri);
        httpd_uri_t save_uri = {};
        save_uri.uri = "/save";
        save_uri.method = HTTP_POST;
        save_uri.handler = save_handler;
        httpd_register_uri_handler(server, &save_uri);
    }
    // Display WiFi instructions with QR code (if LCD is present)
    char instruction_line1[64];
    char instruction_line2[64];
    snprintf(instruction_line1, sizeof(instruction_line1), "connect to wifi:");
    snprintf(instruction_line2, sizeof(instruction_line2), "%s", ap_ssid);
    ESP_LOGI(TAG, "Displaying setup screen (if LCD present)");
    if (qr_display_on_lcd(lcd, instruction_line1, instruction_line2, "http://192.168.4.1")) {
        ESP_LOGI(TAG, "Setup screen displayed on LCD");
    } else {
        ESP_LOGI(TAG, "LCD not available - connect to %s and browse to http://192.168.4.1", ap_ssid);
    }
    return ESP_OK;
}
void wifi_prov_reset(void)
{
    ESP_LOGI(TAG, "Resetting WiFi credentials");
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    esp_restart();
}
