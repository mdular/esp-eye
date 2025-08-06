#pragma once

#include "esp_err.h"

/**
 * @brief Initialize WiFi connection
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_connect_init(void);

/**
 * @brief Start WiFi connection process
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_connect_start(void);

/**
 * @brief Disconnect from WiFi
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_disconnect(void);

/**
 * @brief Check if WiFi is connected
 * 
 * @return true If connected
 * @return false If not connected
 */
bool wifi_is_connected(void);

/**
 * @brief Get WiFi IP address
 * 
 * @return char* IP address string
 */
char* wifi_get_ip_address(void);
