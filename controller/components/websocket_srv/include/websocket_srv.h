#pragma once

#include "esp_err.h"
#include "common_types.h"

/**
 * @brief Initialize WebSocket server
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t websocket_srv_init(void);

/**
 * @brief Start WebSocket server task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t websocket_srv_start(void);

/**
 * @brief Publish telemetry data to WebSocket clients
 *
 * @param data Telemetry data to publish
 * @return esp_err_t ESP_OK on success
 */
esp_err_t websocket_srv_publish_telemetry(const telemetry_data_t *data);

/**
 * @brief Publish log message to WebSocket clients
 *
 * @param log Log message to publish
 * @return esp_err_t ESP_OK on success
 */
esp_err_t websocket_srv_publish_log(const log_message_t *log);