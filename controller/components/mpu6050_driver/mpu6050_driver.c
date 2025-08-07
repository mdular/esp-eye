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
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(i2c_num, MPU6050_ADDR, buf, 2, 1000 / portTICK_PERIOD_MS);
}

// Helper function to read a register
static esp_err_t mpu6050_read_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(i2c_num, MPU6050_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    ESP_LOGI(TAG, "Initializing MPU6050...");

    // Legacy ESP-IDF I2C initialization
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0));

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

#define MADGWICK_BETA 0.1f // Filter gain
#define SAMPLE_FREQ 200.0f // 200Hz sample rate

// Current quaternion state
static quaternion_t current_q = {1.0f, 0.0f, 0.0f, 0.0f}; // Initialize with identity quaternion
static float invSampleFreq = 1.0f / SAMPLE_FREQ;

// Helper functions for Madgwick filter
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

quaternion_t madgwick_update(const mpu_data_t *raw) {
    float gx, gy, gz, ax, ay, az;
    float q0 = current_q.q0, q1 = current_q.q1, q2 = current_q.q2, q3 = current_q.q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope readings to rad/s
    gx = raw->gyro_x * 0.00026646f; // 250dps full scale range / 32768 (16-bit)
    gy = raw->gyro_y * 0.00026646f;
    gz = raw->gyro_z * 0.00026646f;

    // Convert accelerometer readings to normalized values
    ax = raw->accel_x;
    ay = raw->accel_y;
    az = raw->accel_z;

    // Normalize accelerometer values
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated calculations
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    // Normalize step magnitude
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Apply feedback step
    qDot1 -= MADGWICK_BETA * s0;
    qDot2 -= MADGWICK_BETA * s1;
    qDot3 -= MADGWICK_BETA * s2;
    qDot4 -= MADGWICK_BETA * s3;

    // Integrate to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    // Normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Update stored quaternion
    current_q.q0 = q0;
    current_q.q1 = q1;
    current_q.q2 = q2;
    current_q.q3 = q3;

    return current_q;
}
