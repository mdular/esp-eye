#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_pm.h"
#include "esp_log.h"

#include "wifi_connect.h"
#include "websocket_srv.h"
#include "controller.h"

static const char *TAG = "main";

void app_main(void)
{
    // Initial delay to allow power stabilization at boot
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Configure power management
    // esp_pm_config_t pm_config = {
    //     .max_freq_mhz = 160,                // Maximum CPU frequency
    //     .min_freq_mhz = 80,                 // Minimum CPU frequency when idle
    //     .light_sleep_enable = true          // Enable light sleep mode
    // };
    // ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    // ESP_LOGI(TAG, "Power management configured for efficiency");
    
    // Additional delay to stabilize power after NVS initialization
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "ESP32 initialized");

    // Mount SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
    
    // Delay after SPIFFS mount
    vTaskDelay(300 / portTICK_PERIOD_MS);

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    ESP_ERROR_CHECK(wifi_connect_init());
    
    // Delay before starting WiFi to prevent power spike
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    ESP_ERROR_CHECK(wifi_connect_start());

    // Wait for WiFi connection before starting services
    // In production, use event group or callback instead of delay
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // Initialize controller
    ESP_LOGI(TAG, "Initializing controller...");
    ESP_ERROR_CHECK(controller_init());
    
    // Delay to allow controller initialization to complete
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Start WebSocket server
    ESP_LOGI(TAG, "Starting WebSocket server...");
    ESP_ERROR_CHECK(websocket_srv_init());
    
    // Delay before starting WebSocket task
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    // Start controller tasks first so the callback can be registered
    ESP_LOGI(TAG, "Starting controller tasks...");
    ESP_ERROR_CHECK(controller_start_tasks());
    
    // Small delay before starting WebSocket server task
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Now start the WebSocket server task which will register the callback
    ESP_ERROR_CHECK(websocket_srv_start());
    
    ESP_LOGI(TAG, "System startup complete");
}
