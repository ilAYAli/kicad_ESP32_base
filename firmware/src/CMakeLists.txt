idf_component_register(
    SRCS "main.cpp" "lcd.cpp" "temperature.cpp" "ble.cpp" "wifi.cpp" "wifi_prov.cpp" "qrcodegen.c" "qr_display.cpp" "webserver.cpp" "ota.cpp" "touch.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_adc 
        bt 
        nvs_flash 
        esp_driver_spi 
        driver 
        esp_http_server 
        esp_wifi 
        esp_netif
        esp_https_ota
        app_update
    EMBED_TXTFILES
        server_cert.pem
)
