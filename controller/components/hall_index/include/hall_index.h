#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Hall sensor timestamp event
 */
typedef struct {
    uint64_t timestamp;  // Timestamp in microseconds
} hall_event_t;

/**
 * @brief Initialize the Hall-effect sensor
 * 
 * @param gpio_pin GPIO pin number for the Hall sensor
 * @return esp_err_t ESP_OK on success
 */
esp_err_t hall_index_init(int gpio_pin);

/**
 * @brief Get the queue handle for Hall sensor events
 * 
 * @return QueueHandle_t Queue handle
 */
QueueHandle_t hall_index_get_queue(void);
