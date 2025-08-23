#pragma once

#include "esp_err.h"
#include "common_types.h"

/*
 * Compile-time switch to enable forwarding ESP-IDF log messages over WebSocket.
 * 0 (default) disables allocation, queueing and network transmission of log frames
 * to reduce WiFi traffic while investigating connectivity drops / flooding.
 * Set to 1 (e.g. via CFLAGS -DENABLE_WS_LOGS=1 or earlier #define) to re-enable.
 */
#ifndef ENABLE_WS_LOGS
#define ENABLE_WS_LOGS 0
#endif

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
 * @brief Publish log message to WebSocket clients.
 *
 * When ENABLE_WS_LOGS == 0 this becomes a no-op returning ESP_OK.
 *
 * @param log Log message to publish
 * @return esp_err_t ESP_OK on success (or no-op)
 */
esp_err_t websocket_srv_publish_log(const log_message_t *log);