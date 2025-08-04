idf_component_register(
    SRCS "websocket_srv.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_http_server" "esp_wifi" "spiffs" "common" "controller"
)
