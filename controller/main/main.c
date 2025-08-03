#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "wifi_connect.h"
#include "http_server.h"
#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_pm.h"
#include "esp_sleep.h"

// Data update manager function prototype
typedef void (*data_update_callback_t)(int counter);
static data_update_callback_t data_update_callback = NULL;

// Register callback for data updates
void register_data_update_callback(data_update_callback_t callback) {
    data_update_callback = callback;
}

// Update counter data
void update_counter_data(int counter) {
    // Notify registered callbacks
    if (data_update_callback) {
        data_update_callback(counter);
    }
}

void counter_loop(void) {
    int counter = 0;
    const TickType_t delay_time = 1000 / portTICK_PERIOD_MS;
    
    while (1) {
        printf("Counter: %d\n", counter);
        
        // Update the counter data through the decoupled interface
        update_counter_data(counter);
        
        // Increment counter
        counter++;
        
        // Give more time for the system to stabilize
        vTaskDelay(delay_time);
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    // Configure brownout detector - increase threshold to avoid false triggers
    // This requires sdkconfig.h to have CONFIG_ESP_BROWNOUT_DET enabled
    // Use esp_brownout_disable() if you want to disable it completely
    // esp_brownout_set_threshold(3); // Set to a higher threshold (default is usually around 2.7V)
    
    // Set CPU frequency to a moderate level to reduce power consumption
    // esp_pm_config_esp32_t pm_config = {
    //     .max_freq_mhz = 160,
    //     .min_freq_mhz = 80,
    //     .light_sleep_enable = false
    // };
    // esp_pm_configure(&pm_config);
    
    // Delay to stabilize power at startup
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    printf("[System] ESP32 initialized\n");

    // Mount SPIFFS
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        printf("[SPIFFS] Failed to mount or format filesystem\n");
    } else {
        printf("[SPIFFS] SPIFFS mounted successfully\n");
    }

    printf("[WiFi] Initializing WiFi...\n");
    wifi_init_sta();

    // Wait for WiFi connection before starting HTTP server
    // In production, use event group or callback to wait for connection
    vTaskDelay(5000 / portTICK_PERIOD_MS); // crude wait for WiFi
    
    // Start HTTP server
    start_http_server();
    
    // Register WebSocket update callback
    register_data_update_callback(update_websocket_data);
    
    // Start the counter loop
    counter_loop();
}