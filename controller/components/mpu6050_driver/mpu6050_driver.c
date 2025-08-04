#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050_driver.h"

static const char *TAG = "mpu6050";

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43

// Helper function to write a byte to a register
static esp_err_t mpu6050_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Helper function to read a register
static esp_err_t mpu6050_read_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    ESP_LOGI(TAG, "Initializing MPU6050...");
    
    // Configure MPU6050
    // Wake up the MPU6050 (0x00 = wake up)
    esp_err_t ret = mpu6050_write_reg(i2c_num, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_get_raw(i2c_port_t i2c_num, mpu_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_reg(i2c_num, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
        return ret;
    }
    
    // Combine high and low bytes
    data->accel_x = (buffer[0] << 8) | buffer[1];
    data->accel_y = (buffer[2] << 8) | buffer[3];
    data->accel_z = (buffer[4] << 8) | buffer[5];
    
    // Temperature
    int16_t temp_raw = (buffer[6] << 8) | buffer[7];
    data->temperature = temp_raw / 340.0f + 36.53f;  // MPU6050 datasheet formula
    
    // Gyroscope
    data->gyro_x = (buffer[8] << 8) | buffer[9];
    data->gyro_y = (buffer[10] << 8) | buffer[11];
    data->gyro_z = (buffer[12] << 8) | buffer[13];
    
    return ESP_OK;
}

quaternion_t madgwick_update(const mpu_data_t *raw) {
    // Simplified implementation - just returns identity quaternion
    // Full Madgwick filter implementation will be added later
    quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}
