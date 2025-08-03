#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "wifi_connect.h"
#include "http_server.h"
#include "esp_event.h"
#include "esp_spiffs.h"

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
    while (1) {
        printf("Counter: %d\n", counter);
        
        // Update the counter data through the decoupled interface
        update_counter_data(counter);
        
        // Increment counter
        counter++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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