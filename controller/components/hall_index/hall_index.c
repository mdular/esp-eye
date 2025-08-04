#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hall_index.h"

static const char *TAG = "hall_index";

// Static variables
static QueueHandle_t hall_queue = NULL;
static int hall_gpio_pin = -1;

// ISR handler
static void IRAM_ATTR hall_isr_handler(void* arg) {
    uint64_t timestamp = esp_timer_get_time();
    hall_event_t event = {
        .timestamp = timestamp
    };
    xQueueSendFromISR(hall_queue, &event, NULL);
}

esp_err_t hall_index_init(int gpio_pin) {
    ESP_LOGI(TAG, "Initializing Hall sensor on GPIO %d", gpio_pin);
    
    hall_gpio_pin = gpio_pin;
    
    // Create event queue
    hall_queue = xQueueCreate(10, sizeof(hall_event_t));
    if (hall_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create hall event queue");
        return ESP_FAIL;
    }
    
    // Configure GPIO for Hall sensor
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // Interrupt on falling edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio_pin),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Hook ISR handler for GPIO pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_pin, hall_isr_handler, NULL));
    
    ESP_LOGI(TAG, "Hall sensor initialized successfully");
    return ESP_OK;
}

QueueHandle_t hall_index_get_queue(void) {
    return hall_queue;
}
