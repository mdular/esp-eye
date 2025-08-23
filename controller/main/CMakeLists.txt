idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES utils websocket_srv controller spiffs esp_pm nvs_flash common)