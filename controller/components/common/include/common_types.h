#pragma once

#include <stdint.h>

// Default runtime toggles for controller activities (all OFF)
#define DEFAULT_MPU6050_ENABLED     0
#define DEFAULT_MOTORS_ENABLED      0
#define DEFAULT_HALL_SENSOR_ENABLED 0

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
 * @brief Orientation sample passed from imu_task → control_task
 *
 * Euler angles are in radians (internal control units).
 */
typedef struct {
    quaternion_t q;
    float roll;              // rad
    float pitch;             // rad
    float yaw;               // rad
    uint64_t sample_time_us; // esp_timer timestamp
    uint32_t seq;            // incremental sample counter
    uint32_t missed_reads;   // cumulative sensor read failures
} orientation_sample_t;

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
    log_level_t level;
    uint64_t timestamp_ms;
    char message[256];
    char source[32];
} log_message_t;

/**
 * @brief Telemetry data structure for communication between components
 *
 * Roll / Pitch / Yaw are exported in degrees for UI consumption.
 *
 * Additional fault / diagnostic counters:
 *  - hall_timeout_count: number of times hall pulses exceeded timeout threshold (no pulse within window)
 *  - motor_sat_count: number of times motor command required saturation (|cmd| > 1.0)
 *  - ctrl_q_depth / orient_q_depth: instantaneous depths of control & orientation queues (diagnostics)
 */
typedef struct {
    // Quaternion (latest fused orientation)
    float q0;
    float q1;
    float q2;
    float q3;

    // IMU Euler (deg)
    float roll;
    float pitch;
    float yaw;
    
    // Hall effect sensor data
    uint64_t last_hall_pulse_us;
    uint32_t hall_timeout_count;   // cumulative hall timeout events
    
    // Motor speeds (commanded, -1..1)
    float motor1_speed;
    float motor2_speed;
    uint32_t motor_sat_count;      // cumulative motor saturation events

    // Queue diagnostics
    uint8_t ctrl_q_depth;
    uint8_t orient_q_depth;

    // Health / status
    uint32_t imu_missed_reads;
    uint32_t imu_sequence;
    int32_t status; // 0 = IDLE, 1 = RUNNING, 2 = ESTOP, etc.

    // Error message for reporting issues
    char error_message[128];
} telemetry_data_t;
