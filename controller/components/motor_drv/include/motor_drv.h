#pragma once

#include "esp_err.h"

/**
 * @brief Motor driver initialization
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_drv_init(void);

/**
 * @brief Set motor speed
 * 
 * @param motor_id Motor ID (0 or 1)
 * @param speed Speed value (-1.0 to 1.0)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_drv_set_speed(int motor_id, float speed);

/**
 * @brief Emergency stop all motors
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_drv_emergency_stop(void);
