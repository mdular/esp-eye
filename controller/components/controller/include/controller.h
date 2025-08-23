#pragma once

#include "esp_err.h"
#include "mpu6050_driver.h"
#include "common_types.h"
#include <stdbool.h>

/**
 * @brief Controller status enum
 */
typedef enum {
    CONTROLLER_IDLE,
    CONTROLLER_RUNNING,
    CONTROLLER_ERROR,
    CONTROLLER_CALIBRATING
} controller_status_t;

/**
 * @brief Telemetry callback function type
 */
typedef esp_err_t (*controller_telemetry_cb_t)(const telemetry_data_t *data);

/**
 * @brief Initialize the main controller
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t controller_init(void);

/**
 * @brief Start controller tasks
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t controller_start_tasks(void);

/**
 * @brief Get current controller status
 *
 * @return controller_status_t Status code
 */
controller_status_t controller_get_status(void);

/**
 * @brief Get current orientation quaternion
 *
 * @return quaternion_t Current orientation
 */
quaternion_t controller_get_orientation(void);

/**
 * @brief Get current telemetry data
 *
 * @param data Pointer to store telemetry data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t controller_get_telemetry(telemetry_data_t *data);

/**
 * @brief Update internal telemetry data
 *
 * @param data Pointer to new telemetry data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t controller_update_telemetry(const telemetry_data_t *data);

/**
 * @brief Register a callback for telemetry data
 *
 * @param callback Function to call when new telemetry data is available
 * @return esp_err_t ESP_OK on success
 */
esp_err_t controller_register_telemetry_callback(controller_telemetry_cb_t callback);

/* Runtime enable/disable toggles */
void controller_set_mpu6050_enabled(bool enabled);
bool controller_get_mpu6050_enabled(void);
void controller_set_hall_sensor_enabled(bool enabled);
bool controller_get_hall_sensor_enabled(void);
void controller_set_motors_enabled(bool enabled);
bool controller_get_motors_enabled(void);

/**
 * @brief Emergency stop: disables motors and commands zero output immediately.
 *
 * Sets telemetry status to 2 (ESTOP) for UI diagnostics.
 */
esp_err_t controller_emergency_stop(void);
