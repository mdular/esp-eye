#pragma once

// Central sensor toggles (set to 1 to enable, 0 to disable)
#define ENABLE_MPU6050     0
#define ENABLE_MOTORS      0
#define ENABLE_HALL_SENSOR 0

/**
 * @brief Quaternion structure for orientation representation
 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion_t;

/**
 * @brief Log level enum following standard logging conventions
 */
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_CRITICAL
} log_level_t;

/**
 * @brief Log message structure for error and status reporting
 */
typedef struct {
    // Log level (debug, info, warning, error, critical)
    log_level_t level;
    
    // Timestamp in milliseconds since boot
    uint64_t timestamp_ms;
    
    // Log message text
    char message[256];
    
    // Optional component/module identifier
    char source[32];
} log_message_t;

/**
 * @brief Telemetry data structure for communication between components
 */
typedef struct {
    // IMU Data
    float roll;
    float pitch;
    float yaw;
    
    // Hall effect sensor data
    uint64_t last_hall_pulse_us;
    
    // Motor speeds
    float motor1_speed;
    float motor2_speed;
    
    // Error message for reporting issues
    char error_message[128];
} telemetry_data_t;
