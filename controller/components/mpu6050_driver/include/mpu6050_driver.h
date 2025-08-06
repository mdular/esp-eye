#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include "common_types.h"

/**
 * @brief MPU6050 data structure
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float temperature;
} mpu_data_t;

/**
 * @brief Initialize the MPU6050 sensor
 * 
 * @param i2c_num I2C port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_init(i2c_port_t i2c_num);

/**
 * @brief Read raw data from MPU6050
 * 
 * @param i2c_num I2C port number
 * @param data Pointer to store the raw data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mpu6050_get_raw(i2c_port_t i2c_num, mpu_data_t *data);

/**
 * @brief Update quaternion using Madgwick filter
 * 
 * @param raw Raw IMU data
 * @return quaternion_t Updated quaternion
 */
quaternion_t madgwick_update(const mpu_data_t *raw);
