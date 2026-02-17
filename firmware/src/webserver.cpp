#include "webserver.hpp"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
static const char *TAG = "WEBSERVER";
static httpd_handle_t server = NULL;
static float current_temp = 0.0f;
static float min_temp = 0.0f;
static float max_temp = 0.0f;
static const char* html_page =
"<!DOCTYPE html>"
"<html><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<style>"
"body{font-family:Arial,sans-serif;background:#1e1e1e;color:#e0e0e0;margin:0;padding:20px;display:flex;justify-content:center;align-items:center;min-height:100vh}"
".container{background:#2d2d2d;border-radius:12px;padding:30px;box-shadow:0 4px 6px rgba(0,0,0,0.3);max-width:400px;width:100%}"
"h1{margin:0 0 20px;font-size:24px;text-align:center;color:#4a9eff}"
".temp-main{font-size:64px;font-weight:bold;text-align:center;margin:20px 0;color:#4a9eff}"
".temp-range{display:flex;justify-content:space-around;margin-top:20px;padding-top:20px;border-top:1px solid #404040}"
".temp-item{text-align:center}"
".temp-label{font-size:12px;color:#888;text-transform:uppercase;margin-bottom:5px}"
".temp-value{font-size:24px;font-weight:bold}"
".min{color:#6ea8ff}.max{color:#ff6b6b}"
".updated{text-align:center;color:#666;font-size:12px;margin-top:20px}"
"</style>"
"</head><body>"
"<div class='container'>"
"<h1>Temperature Monitor</h1>"
"<div class='temp-main' id='current'>--</div>"
"<div class='temp-range'>"
"<div class='temp-item'><div class='temp-label'>Min</div><div class='temp-value min' id='min'>--</div></div>"
"<div class='temp-item'><div class='temp-label'>Max</div><div class='temp-value max' id='max'>--</div></div>"
"</div>"
"<div class='updated' id='updated'>Updating...</div>"
"</div>"
"<script>"
"function update(){"
"fetch('/api/temperature').then(r=>r.json()).then(d=>{"
"document.getElementById('current').textContent=d.temperature.toFixed(1)+'°C';"
"document.getElementById('min').textContent=d.min.toFixed(1)+'°C';"
"document.getElementById('max').textContent=d.max.toFixed(1)+'°C';"
"document.getElementById('updated').textContent='Updated: '+new Date().toLocaleTimeString();"
"}).catch(()=>document.getElementById('updated').textContent='Connection error');}"
"update();setInterval(update,10000);"
"</script>"
"</body></html>";
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_page, strlen(html_page));
}
static esp_err_t temp_get_handler(httpd_req_t *req)
{
    char json[256];
    snprintf(json, sizeof(json),
             "{\"temperature\":%.2f,\"min\":%.2f,\"max\":%.2f,\"unit\":\"C\"}",
             current_temp, min_temp, max_temp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}
static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};
static const httpd_uri_t temp_uri = {
    .uri       = "/api/temperature",
    .method    = HTTP_GET,
    .handler   = temp_get_handler,
    .user_ctx  = NULL
};
esp_err_t webserver_start(void)
{
    if (server != NULL) {
        ESP_LOGW(TAG, "Server already running");
        return ESP_OK;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &temp_uri);
        ESP_LOGI(TAG, "Web server started on port %d", config.server_port);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to start web server");
    return ESP_FAIL;
}
void webserver_stop(void)
{
    if (server) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
}
void webserver_update_temp_data(float current, float min, float max)
{
    current_temp = current;
    min_temp = min;
    max_temp = max;
}
bool webserver_is_running(void)
{
    return server != NULL;
}
