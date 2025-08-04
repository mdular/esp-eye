# Generate wifi_secrets.h from wifi_secrets.conf in main directory
set(WIFI_SECRETS_CONF ${CMAKE_SOURCE_DIR}/main/wifi_secrets.conf)
set(WIFI_SECRETS_H ${CMAKE_CURRENT_SOURCE_DIR}/include/wifi_secrets.h)

if(EXISTS ${WIFI_SECRETS_CONF})
    file(READ ${WIFI_SECRETS_CONF} WIFI_CONF_CONTENTS)
    string(REGEX MATCH "WIFI_SSID=([^\n]*)" _ssid_match "${WIFI_CONF_CONTENTS}")
    string(REGEX REPLACE ".*WIFI_SSID=([^\n]*).*" "\\1" WIFI_SSID "${WIFI_CONF_CONTENTS}")
    string(REGEX MATCH "WIFI_PASS=([^\n]*)" _pass_match "${WIFI_CONF_CONTENTS}")
    string(REGEX REPLACE ".*WIFI_PASS=([^\n]*).*" "\\1" WIFI_PASS "${WIFI_CONF_CONTENTS}")
    file(WRITE ${WIFI_SECRETS_H} "// Auto-generated from wifi_secrets.conf\n")
    file(APPEND ${WIFI_SECRETS_H} "#define WIFI_SSID \"${WIFI_SSID}\"\n")
    file(APPEND ${WIFI_SECRETS_H} "#define WIFI_PASS \"${WIFI_PASS}\"\n")
endif()

idf_component_register(
    SRCS "wifi_connect.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_wifi" "nvs_flash" "esp_netif"
)
