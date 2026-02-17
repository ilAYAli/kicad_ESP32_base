#include "ble.hpp"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
static const char *TAG = "BT";
static uint16_t temp_handle;
static float current_temp = 0;
static uint8_t addr_type;
static bool bt_connected = false;
static uint16_t conn_handle = 0;
static int8_t bt_rssi = -100;
static bool advertising_enabled = true;
static uint32_t advertising_start_time = 0;
static const uint32_t ADVERTISING_TIMEOUT_MS = 180000;  // 3 minutes
static bool ble_initialized = false;
static bool ever_connected = false;
static char mac_str[18] = "00:00:00:00:00:00";
static char adv_name[] = "pwa-00:00:00:00:00:00";
static void bt_start_advertising(void);
static int bt_gap_event(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        bt_connected = (event->connect.status == 0);
        ESP_LOGI(TAG, "Bluetooth %s", bt_connected ? "CONNECTED" : "CONNECTION FAILED");
        if (bt_connected) {
            conn_handle = event->connect.conn_handle;
            advertising_enabled = true;  // Reset timeout on successful connection
            ever_connected = true;  // Track that we've had at least one connection
            // Read initial RSSI
            ble_gap_conn_rssi(conn_handle, &bt_rssi);
        }
    } else if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        bt_connected = false;
        bt_rssi = -100;
        ESP_LOGI(TAG, "Bluetooth disconnected");
        // Only restart advertising if still within timeout period
        if (advertising_enabled) {
            uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - advertising_start_time;
            if (elapsed < ADVERTISING_TIMEOUT_MS) {
                ESP_LOGI(TAG, "Restarting advertising (%lu ms remaining)", ADVERTISING_TIMEOUT_MS - elapsed);
                bt_start_advertising();
            } else {
                ESP_LOGI(TAG, "Advertising timeout reached - disabling BLE to save power");
                advertising_enabled = false;
            }
        } else {
            ESP_LOGI(TAG, "Advertising disabled - BLE will not reconnect");
        }
    }
    return 0;
}
static int gatt_read_temp(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return os_mbuf_append(ctxt->om, &current_temp, sizeof(current_temp));
}
// UUID declarations need to be static in C++ to avoid taking address of rvalue
static ble_uuid16_t temp_service_uuid = BLE_UUID16_INIT(0x181A);
static ble_uuid16_t temp_char_uuid = BLE_UUID16_INIT(0x2A6E);

// Suppress warnings for NimBLE struct initializers (standard pattern for sentinel terminators)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &temp_service_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = &temp_char_uuid.u,
          .access_cb = gatt_read_temp,
          .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
          .val_handle = &temp_handle},
         {}}},
    {}
};
#pragma GCC diagnostic pop
static void bt_start_advertising(void)
{
    // Check if advertising is still enabled
    if (!advertising_enabled) {
        ESP_LOGI(TAG, "Advertising disabled - skipping");
        return;
    }
    // Check timeout
    if (advertising_start_time != 0) {
        uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - advertising_start_time;
        if (elapsed >= ADVERTISING_TIMEOUT_MS) {
            ESP_LOGI(TAG, "Advertising timeout reached - disabling BLE");
            advertising_enabled = false;
            return;
        }
    } else {
        advertising_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    struct ble_gap_ext_adv_params params = {};
    int rc;
    params.connectable = 1;
    params.own_addr_type = addr_type;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_1M;
    params.sid = 1;
    rc = ble_gap_ext_adv_configure(0, &params, NULL, bt_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Bluetooth advertising config failed: %d", rc);
        return;
    }
    struct ble_hs_adv_fields fields = {};
    fields.name = (uint8_t *)adv_name;
    fields.name_len = strlen(adv_name);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    struct os_mbuf *adv_data = os_msys_get_pkthdr(BLE_HS_ADV_MAX_SZ, 0);
    if (adv_data) {
        ble_hs_adv_set_fields_mbuf(&fields, adv_data);
        ble_gap_ext_adv_set_data(0, adv_data);
        rc = ble_gap_ext_adv_start(0, 0, 0);
        if (rc == 0) {
            ESP_LOGI(TAG, "Bluetooth advertising started: %s", adv_name);
        } else {
            ESP_LOGE(TAG, "Bluetooth advertising start failed: %d", rc);
        }
    }
}
void ble_host_task(void *param)
{
    nimble_port_run();
}
static void bt_on_sync(void)
{
    ble_hs_id_infer_auto(0, &addr_type);
    uint8_t addr[6];
    int rc = ble_hs_id_copy_addr(addr_type, addr, NULL);
    if (rc == 0) {
        snprintf(mac_str, sizeof(mac_str),
                "%02x:%02x:%02x:%02x:%02x:%02x",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
        snprintf(adv_name, sizeof(adv_name), "pwa-%s", mac_str);
        ESP_LOGI(TAG, "Bluetooth advertising name: %s", adv_name);
    }
    bt_start_advertising();
}
esp_err_t bt_init(void)
{
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = bt_on_sync;
    nimble_port_freertos_init(ble_host_task);
    ble_initialized = true;
    advertising_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    ESP_LOGI(TAG, "BLE initialization complete, advertising timeout: %lu ms", ADVERTISING_TIMEOUT_MS);
    return ESP_OK;
}
void bt_update_temp(float temp)
{
    current_temp = temp;
    if (temp_handle != 0) {
        ble_gatts_chr_updated(temp_handle);
    }
}
bool bt_is_connected(void)
{
    return bt_connected;
}
const char* bt_get_mac_address(void)
{
    return mac_str;
}
const char* bt_get_id(void)
{
    return adv_name;
}
int8_t bt_get_rssi(void)
{
    if (bt_connected) {
        // Update RSSI from connection
        ble_gap_conn_rssi(conn_handle, &bt_rssi);
    }
    return bt_rssi;
}

bool bt_should_shutdown(void)
{
    // Shutdown if initialized, timeout reached, advertising disabled, and never connected
    if (!ble_initialized || ever_connected) {
        return false;
    }
    uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - advertising_start_time;
    return (elapsed >= ADVERTISING_TIMEOUT_MS && !advertising_enabled);
}

void bt_shutdown(void)
{
    if (!ble_initialized) {
        return;
    }
    ESP_LOGI(TAG, "Shutting down BLE to save power");
    nimble_port_stop();
    nimble_port_deinit();
    ble_initialized = false;
    ESP_LOGI(TAG, "BLE shutdown complete");
}
