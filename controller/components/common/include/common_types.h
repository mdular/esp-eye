#pragma once

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
    
    // Counter for testing
    uint32_t counter;
} telemetry_data_t;
