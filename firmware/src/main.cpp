#include <cstdio>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "temperature.hpp"
#include "lcd.hpp"
#include "ble.hpp"
#include "wifi.hpp"
#include "wifi_prov.hpp"
#include "webserver.hpp"
#include "ota.hpp"
#include "touch.hpp"
#include "nvs.h"
#include "pin_config.hpp"

static constexpr int HISTORY_LEN = 280;
static constexpr int POWER_SAVE_INTERVAL_MS = 60000;  // 60 seconds when LCD not present
static constexpr int NORMAL_INTERVAL_MS = 500;        // 0.5 seconds with LCD (UI cadence)
static constexpr int GRAPH_UPDATE_INTERVAL_MS = 1000; // Throttle graph redraws to reduce flicker
extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    // Initialize event loop FIRST (needed by bluetooth and WiFi)
    esp_event_loop_create_default();
    temp_sensor_init();
    // Try to initialize LCD - it might not be present
    LCD lcd;
    bool has_lcd = lcd.isValid();
    // Configure BOOT button for WiFi reset
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << static_cast<int>(Pin::BOOT));
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    // Check if we have WiFi credentials BEFORE initializing
    bool has_wifi = wifi_prov_has_credentials();
    // Get initial temperature reading
    float initial_temp = temp_sensor_read();
    // Initialize screen FIRST so it's visible immediately
    if (has_lcd) {
        lcd.setBrightness(16);
        lcd.screenInit();

        // Initialize touch controller with main task handle for immediate wake-up
        if (touch_init(xTaskGetCurrentTaskHandle()) == ESP_OK) {
            ESP_LOGI("MAIN", "Touch controller initialized");
        } else {
            ESP_LOGW("MAIN", "Touch controller not available");
        }

        if (has_wifi) {
            lcd.headerUpdate("WiFi Connecting...", "", false, false);
        } else {
            lcd.headerUpdate("No WiFi", "Hold BOOT for setup", false, false);
        }
        lcd.statusUpdate("WiFi RSSI:", "-- dBm", C_TEXT_DIM);
        lcd.bodyInit();
        // Show initial temperature immediately
        long long version = OTA_CURRENT_VERSION;
        int mmdd = (version / 10000) % 10000;
        int hhmm = version % 10000;
        char version_str[16];
        snprintf(version_str, sizeof(version_str), "v.%04d%04d", mmdd, hhmm);
        lcd.footerUpdate(initial_temp, initial_temp, initial_temp, "°C", version_str);
    } else {
        ESP_LOGI("MAIN", "LCD not detected - running without display");
    }

    // Power save mode: reduce update frequency when no LCD present
    bool enable_power_save = !has_lcd;
    if (enable_power_save) {
        ESP_LOGI("MAIN", "Power save mode ENABLED - updating every 60 seconds");
    }

    // Init WiFi AFTER screen is visible (BLE will start after WiFi connects)
    wifi_init(has_lcd ? &lcd : nullptr);
    LCDGraph rssi_graph(HISTORY_LEN);
    // DON'T initialize screen yet - let QR code show if provisioning
    // We'll init screen once WiFi connects
    long long version = OTA_CURRENT_VERSION;
    int mmdd = (version / 10000) % 10000;
    int hhmm = version % 10000;
    char version_str[16];
    snprintf(version_str, sizeof(version_str), "v.%04d%04d", mmdd, hhmm);
    bool rssi_initialized = false;
    bool temp_initialized = false;
    bool ota_was_in_progress = false;
    bool ota_started = false;
    bool ble_started = false;
    bool provisioning_mode_active = false;
    bool button_countdown_shown = false;
    bool show_ble_rssi = false;  // false = WiFi RSSI, true = BLE RSSI
    int last_countdown_value = -1;
    float min_temp = 0.0f;
    float max_temp = 0.0f;
    uint32_t button_press_start = 0;

    // Touch marker state - track multiple markers to handle phantom presses
    constexpr int MAX_TOUCH_MARKERS = 10;
    struct TouchMarker {
        int x;
        int y;
        TickType_t time;
        bool active;
    };
    TouchMarker touch_markers[MAX_TOUCH_MARKERS] = {};

    // Start BLE immediately to avoid startup delay
    ESP_LOGI("MAIN", "Starting BLE...");
    bt_init();
    ble_started = true;

    while (1) {
        // Check BOOT button for provisioning mode (hold >4 seconds)
        int button_level = gpio_get_level(static_cast<gpio_num_t>(Pin::BOOT));
        if (button_level == 0) {
            if (button_press_start == 0) {
                button_press_start = xTaskGetTickCount();
                ESP_LOGI("MAIN", "BOOT button pressed - hold for 4 seconds for WiFi setup");
            }
            uint32_t hold_time = (xTaskGetTickCount() - button_press_start) * portTICK_PERIOD_MS;
            // Hold for >4 seconds to enter provisioning mode
            if (hold_time >= 4000 && !provisioning_mode_active) {
                ESP_LOGI("MAIN", "Starting provisioning mode!");
                // Set flag FIRST to prevent footer from being drawn
                provisioning_mode_active = true;
                if (has_lcd) {
                    lcd.showMessage("Starting WiFi Setup", -1);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                // Clear any existing credentials
                nvs_handle_t h;
                if (nvs_open("wifi_config", NVS_READWRITE, &h) == ESP_OK) {
                    nvs_erase_all(h);
                    nvs_commit(h);
                    nvs_close(h);
                }
                // Start provisioning AP
                wifi_prov_start_ap_mode(has_lcd ? &lcd : nullptr);
                button_press_start = 0;
                button_countdown_shown = false;
                last_countdown_value = -1;
            } else if (hold_time < 4000 && hold_time >= 1000) {
                // Show countdown - calculate remaining seconds (only for holds > 1s)
                int remaining = (4000 - hold_time) / 1000;
                if (remaining != last_countdown_value) {
                    if (has_lcd) {
                        char msg[32];
                        snprintf(msg, sizeof(msg), "WiFi Setup in %d", remaining);
                        lcd.showMessage(msg, -1);
                        button_countdown_shown = true;
                    }
                    last_countdown_value = remaining;
                    ESP_LOGI("MAIN", "Countdown: %d", remaining);
                }
            }
        } else {
            if (button_press_start != 0) {
                uint32_t hold_time = (xTaskGetTickCount() - button_press_start) * portTICK_PERIOD_MS;
                ESP_LOGI("MAIN", "Button released after %lu ms, countdown_shown=%d, ble_started=%d", 
                         hold_time, button_countdown_shown, ble_started);
                // Short press (<1s) = toggle RSSI display (only if BLE is started)
                if (hold_time < 1000 && !button_countdown_shown) {
                    if (ble_started) {
                        show_ble_rssi = !show_ble_rssi;
                        rssi_initialized = false;  // Force graph redraw
                        ESP_LOGI("MAIN", "Toggled to %s RSSI display", show_ble_rssi ? "BLE" : "WiFi");
                    } else {
                        ESP_LOGI("MAIN", "BLE not started yet - cannot toggle");
                    }
                }
                // Redraw the screen if countdown was shown
                if (has_lcd && button_countdown_shown && !provisioning_mode_active) {
                    lcd.screenInit();
                    bool has_wifi = wifi_prov_has_credentials();
                    if (has_wifi) {
                        lcd.headerUpdate("WiFi Connecting...", "", false, false);
                    } else {
                        lcd.headerUpdate("No WiFi", "Hold BOOT for setup", false, false);
                    }
                    lcd.statusUpdate("WiFi RSSI:", "-- dBm", C_TEXT_DIM);
                    lcd.bodyInit();
                }
            }
            button_press_start = 0;
            button_countdown_shown = false;
            last_countdown_value = -1;
        }
        bool ota_active = ota_is_in_progress();
        if (ota_was_in_progress && !ota_active) {
            ESP_LOGI("MAIN", "OTA completed, reinitializing display...");
            if (has_lcd) {
                lcd.screenInit();
                lcd.headerUpdate("WiFi Connected", "", false, false);
                lcd.statusUpdate("WiFi RSSI:", "-- dBm", C_TEXT_DIM);
                lcd.bodyInit();
            }
            rssi_initialized = false;
        }
        ota_was_in_progress = ota_active;
        if (ota_active) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        // Don't update display if in provisioning mode
        if (provisioning_mode_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Check WiFi connection and start OTA immediately if connected
        bool wifi_state = wifi_is_connected();
        if (wifi_state && !ota_started) {
            ESP_LOGI("MAIN", "Starting OTA background check...");
            ota_start_background_check(has_lcd ? &lcd : nullptr);
            ota_started = true;
        }

        // Check if BLE should shutdown due to timeout with no connections
        if (ble_started && bt_should_shutdown()) {
            ESP_LOGI("MAIN", "BLE timeout reached with no connections, shutting down to save power");
            bt_shutdown();
            ble_started = false;
        }

        // Check for touch events (IRQ-driven)
        TickType_t current_time = xTaskGetTickCount();
        if (has_lcd && touch_has_event()) {
            TouchPoint touch;
            if (touch_read(&touch) && touch.touched) {
                // Raw coordinate jump detection in touch_read() handles phantom filtering
                ESP_LOGI("MAIN", "Touch at (%d, %d)", touch.x, touch.y);
                // Draw a larger fingerprint circle at touch location
                lcd.drawFilledCircle(touch.x, touch.y, 10, C_GLOW); // Light blue, 10px radius
                // Store marker in first available slot
                for (int i = 0; i < MAX_TOUCH_MARKERS; i++) {
                    if (!touch_markers[i].active) {
                        touch_markers[i].x = touch.x;
                        touch_markers[i].y = touch.y;
                        touch_markers[i].time = current_time;
                        touch_markers[i].active = true;
                        break;
                    }
                }
            }
        }

        // Clear touch markers after 1 second
        TickType_t now = xTaskGetTickCount();
        for (int i = 0; i < MAX_TOUCH_MARKERS; i++) {
            if (touch_markers[i].active && (now - touch_markers[i].time) >= pdMS_TO_TICKS(1000)) {
                lcd.drawFilledCircle(touch_markers[i].x, touch_markers[i].y, 10, C_PANEL_DK);
                touch_markers[i].active = false;
            }
        }

        static TickType_t last_ui_tick = 0;
        // Adjust update frequency based on power save mode
        // If waiting for OTA to start, check more frequently (1s) to catch WiFi connection quickly
        uint32_t update_interval_ms;
        if (enable_power_save && !ota_started) {
            update_interval_ms = 1000;  // Check every second until OTA starts
        } else {
            update_interval_ms = enable_power_save
                ? POWER_SAVE_INTERVAL_MS
                : NORMAL_INTERVAL_MS;
        }
        TickType_t now_tick = xTaskGetTickCount();
        TickType_t update_interval_ticks = pdMS_TO_TICKS(update_interval_ms);
        if (last_ui_tick == 0) {
            last_ui_tick = now_tick - update_interval_ticks;
        }
        TickType_t elapsed_ticks = now_tick - last_ui_tick;
        if (elapsed_ticks < update_interval_ticks) {
            TickType_t remaining_ticks = update_interval_ticks - elapsed_ticks;
            xTaskNotifyWait(0, 0, nullptr, remaining_ticks);
            continue;
        }
        last_ui_tick = now_tick;

        static TickType_t last_graph_tick = 0;
        bool graph_update_due = false;
        if (last_graph_tick == 0) {
            last_graph_tick = now_tick - pdMS_TO_TICKS(GRAPH_UPDATE_INTERVAL_MS);
        }
        if (now_tick - last_graph_tick >= pdMS_TO_TICKS(GRAPH_UPDATE_INTERVAL_MS)) {
            graph_update_due = true;
            last_graph_tick = now_tick;
        }

        float temp = temp_sensor_read();
        int8_t rssi = show_ble_rssi ? bt_get_rssi() : wifi_get_rssi();
        bool bt_state = bt_is_connected();

        if (!temp_initialized && temp > -50.0f && temp < 100.0f) {
            min_temp = temp;
            max_temp = temp;
            temp_initialized = true;
        }
        if (temp_initialized) {
            if (temp < min_temp) min_temp = temp;
            if (temp > max_temp) max_temp = temp;
        }
        if (wifi_state) {
            if (!rssi_initialized) {
                if (has_lcd) {
                    for (int i = 0; i < HISTORY_LEN; i++) {
                        rssi_graph.addSample((float)rssi);
                    }
                }
                rssi_initialized = true;
                graph_update_due = true;
            }
            if (has_lcd && graph_update_due) {
                rssi_graph.addSample((float)rssi);
            }
        } else if (show_ble_rssi && bt_state) {
            // If showing BLE RSSI and BLE is connected, update graph
            if (!rssi_initialized) {
                if (has_lcd) {
                    for (int i = 0; i < HISTORY_LEN; i++) {
                        rssi_graph.addSample((float)rssi);
                    }
                }
                rssi_initialized = true;
                graph_update_due = true;
            }
            if (has_lcd && graph_update_due) {
                rssi_graph.addSample((float)rssi);
            }
        }

        // Only update BLE if it was initialized
        if (ble_started) {
            bt_update_temp(temp);
        }

        webserver_update_temp_data(temp, min_temp, max_temp);
        if (has_lcd) {
            const char *ip = wifi_get_ip_address();
            const char *bt_id = bt_get_id();
            char header_line1[64];
            char header_line2[32];
            snprintf(header_line1, sizeof(header_line1), "IP: %s:80", ip);
            snprintf(header_line2, sizeof(header_line2), "BT: %s", bt_id);
            lcd.headerUpdate(header_line1, header_line2, wifi_state, bt_state);
            char rssi_str[32];
            uint16_t rssi_color = C_WARN;
            if (rssi >= -70) rssi_color = C_GOOD;
            if (rssi >= -50) rssi_color = C_ACCENT;
            snprintf(rssi_str, sizeof(rssi_str), "%d dBm", rssi);
            const char* rssi_label = show_ble_rssi ? "BT RSSI:" : "WiFi RSSI:";
            lcd.statusUpdate(rssi_label, rssi_str, rssi_color);
            if (rssi_initialized && graph_update_due) {
                lcd.bodyDrawGraph(&rssi_graph);
            }
            if (temp_initialized) {
                lcd.footerUpdate(temp, min_temp, max_temp, "C", version_str);
            }
        }

        // Use notification to wake immediately on touch, but still sleep normally otherwise
        xTaskNotifyWait(0, 0, nullptr, update_interval_ticks);
    }
}
