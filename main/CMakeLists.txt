idf_component_register(SRCS "ssd1306.c" "main.c" "dht11.c" "rest_client.c" "adxl345.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_client mqtt cjson esp_driver_i2c esp_driver_gpio esp_driver_uart driver esp_timer esp_adc esp_driver_ledc)