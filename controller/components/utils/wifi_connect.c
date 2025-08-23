#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "wifi_connect.h"

static const char *TAG = "wifi";
static const int WIFI_TX_POWER_DBM = 50; // Range is 8-84, which maps to power levels from 2dBm to 20dBm
static esp_netif_t *s_sta_netif = NULL;
static bool s_wifi_connected = false;
static char s_ip_address[16] = {0};

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Connecting to AP...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Reconnecting...");
        s_wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        snprintf(s_ip_address, sizeof(s_ip_address), IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// WiFi credentials are injected at build time via wifi_secrets.h (auto-generated from wifi_secrets.conf)
#include "wifi_secrets.h"

esp_err_t wifi_connect_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                        ESP_EVENT_ANY_ID,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                        IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler,
                                        NULL,
                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    
    // Configure WiFi with power-saving features
    // Set WiFi power save mode
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    
    ESP_LOGI(TAG, "WiFi initialization complete with power-saving settings");
    return ESP_OK;
}

esp_err_t wifi_connect_start(void) {
    // Delay before starting WiFi to stabilize power
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Wait for WiFi to initialize before setting power
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Set the output power level (lower power to reduce current draw)
    // Lower value = less power consumption but shorter range
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER_DBM)); // Set WiFi TX power via constant
    
    ESP_LOGI(TAG, "WiFi connection started with minimal power");
    return ESP_OK;
}

esp_err_t wifi_disconnect(void) {
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    s_wifi_connected = false;
    ESP_LOGI(TAG, "WiFi disconnected");
    return ESP_OK;
}

bool wifi_is_connected(void) {
    return s_wifi_connected;
}

char* wifi_get_ip_address(void) {
    return s_ip_address;
}
