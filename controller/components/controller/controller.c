#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "controller.h"
#include "mpu6050_driver.h"
#include "hall_index.h"
#include "common_types.h"
#include <math.h>

static const char *TAG = "controller";

// Runtime toggles for controller activities (default OFF)
static bool mpu6050_enabled = DEFAULT_MPU6050_ENABLED;
static bool hall_sensor_enabled = DEFAULT_HALL_SENSOR_ENABLED;
static bool motors_enabled = DEFAULT_MOTORS_ENABLED;

// Setter/getter functions
void controller_set_mpu6050_enabled(bool enabled) { mpu6050_enabled = enabled; }
bool controller_get_mpu6050_enabled(void) { return mpu6050_enabled; }
void controller_set_hall_sensor_enabled(bool enabled) { hall_sensor_enabled = enabled; }
bool controller_get_hall_sensor_enabled(void) { return hall_sensor_enabled; }
void controller_set_motors_enabled(bool enabled) { motors_enabled = enabled; }
bool controller_get_motors_enabled(void) { return motors_enabled; }

// Task parameters
#define IMU_TASK_STACK_SIZE 3072
#define CONTROL_TASK_STACK_SIZE 3072
#define MOTOR_TASK_STACK_SIZE 3072

#define IMU_TASK_PRIORITY 6
#define CONTROL_TASK_PRIORITY 6
#define MOTOR_TASK_PRIORITY 5

// Core assignments
#define IMU_TASK_CORE 0        // Real-time core
#define CONTROL_TASK_CORE 0     // Real-time core
#define MOTOR_TASK_CORE 0       // Real-time core

// Task handles
static TaskHandle_t imu_task_handle = NULL;
static TaskHandle_t control_task_handle = NULL;
static TaskHandle_t motor_task_handle = NULL;

// Queue handles
static QueueHandle_t imu_queue = NULL;
static QueueHandle_t control_queue = NULL;

// Static queue buffers and storage areas
static StaticQueue_t imu_queue_buffer;
static uint8_t imu_queue_storage[sizeof(mpu_data_t)];

static StaticQueue_t control_queue_buffer;
static uint8_t control_queue_storage[sizeof(float) * 2];


// Telemetry callback
static controller_telemetry_cb_t telemetry_callback = NULL;

// Global telemetry data storage
static telemetry_data_t global_telemetry = {
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .last_hall_pulse_us = 0,
    .motor1_speed = 0.0f,
    .motor2_speed = 0.0f,
};

// Task prototypes
static void imu_task(void *pvParameters);
static void control_task(void *pvParameters);
static void motor_task(void *pvParameters);

// Helper function prototypes
esp_err_t i2c_master_init(void);

esp_err_t controller_init(void) {
    // Create queues for inter-task communication using static allocation
    imu_queue = xQueueCreateStatic(1, sizeof(mpu_data_t), imu_queue_storage, &imu_queue_buffer);
    if (imu_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create imu_queue");
        return ESP_FAIL;
    }

    control_queue = xQueueCreateStatic(1, sizeof(float) * 2, control_queue_storage, &control_queue_buffer);
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control_queue");
        return ESP_FAIL;
    }


    ESP_LOGI(TAG, "Controller initialized successfully");
    return ESP_OK;
}

esp_err_t controller_start_tasks(void) {
    BaseType_t ret;
    esp_err_t err;
    
    // Initialize I2C for IMU
    err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(err));
        // Log error but don't store in telemetry
        // Continue anyway to allow web interface to work
    }
    
if (mpu6050_enabled) {
    // Initialize MPU6050
    err = mpu6050_init(I2C_NUM_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(err));
        // Log error but don't store in telemetry
        // Continue anyway to allow web interface to work
    }
}

if (hall_sensor_enabled) {
    // Initialize Hall-effect sensor on GPIO 4
    err = hall_index_init(4);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Hall sensor: %s", esp_err_to_name(err));
        // Log error but don't store in telemetry
        // Continue anyway to allow web interface to work
    }
}
    
    // Allow some time for I2C devices to stabilize
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
if (mpu6050_enabled) {
    // Create IMU task
    ret = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        &imu_task_handle,
        IMU_TASK_CORE
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create imu_task");
        return ESP_FAIL;
    }
}
    
    // Allow some time for IMU task to start
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Create control task
    ret = xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        &control_task_handle,
        CONTROL_TASK_CORE
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control_task");
        return ESP_FAIL;
    }
    
    // Allow some time for control task to start
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Create motor task
    ret = xTaskCreatePinnedToCore(
        motor_task,
        "motor_task",
        MOTOR_TASK_STACK_SIZE,
        NULL,
        MOTOR_TASK_PRIORITY,
        &motor_task_handle,
        MOTOR_TASK_CORE
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor_task");
        return ESP_FAIL;
    }
    
    
    ESP_LOGI(TAG, "All controller tasks started successfully");
    return ESP_OK;
}

controller_status_t controller_get_status(void) {
    // Simple status indication for now
    if (imu_task_handle != NULL && control_task_handle != NULL) {
        return CONTROLLER_RUNNING;
    }
    return CONTROLLER_IDLE;
}

quaternion_t controller_get_orientation(void) {
    // Placeholder for now
    quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

esp_err_t controller_get_telemetry(telemetry_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy from global telemetry data
    memcpy(data, &global_telemetry, sizeof(telemetry_data_t));
    
    // Make sure counter is always up to date
    return ESP_OK;
}

esp_err_t controller_update_telemetry(const telemetry_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy data to global telemetry structure
    memcpy(&global_telemetry, data, sizeof(telemetry_data_t));
    
    return ESP_OK;
}

esp_err_t controller_register_telemetry_callback(controller_telemetry_cb_t callback) {
    telemetry_callback = callback;
    return ESP_OK;
}

// IMU task - samples IMU at 200Hz and puts data in queue
static void imu_task(void *pvParameters) {
    // ... (function unchanged)
}

// Control task - processes IMU data and sends motor commands
static void control_task(void *pvParameters) {
    // Initialize the last wake time to the current tick count
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // 5ms period (200Hz) as required by architecture
    const TickType_t period = pdMS_TO_TICKS(5);
    
    ESP_LOGI(TAG, "Control task started");
    
    // Delay for one period before starting the loop to ensure stability
    vTaskDelay(period);
    
    // Reset the last wake time after the initial delay
    last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Get latest IMU data
        mpu_data_t imu_data;
        
        if (xQueueReceive(imu_queue, &imu_data, 0) == pdTRUE) {
            // Get the updated quaternion from the IMU data
            quaternion_t q = madgwick_update(&imu_data);
            
            // Extract Euler angles from quaternion
            float roll, pitch, yaw;
            
            // Convert quaternion to Euler angles
            // roll (x-axis rotation)
            float sinr_cosp = 2.0f * (q.q0 * q.q1 + q.q2 * q.q3);
            float cosr_cosp = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);
            roll = atan2f(sinr_cosp, cosr_cosp);
            
            // pitch (y-axis rotation)
            float sinp = 2.0f * (q.q0 * q.q2 - q.q3 * q.q1);
            if (fabsf(sinp) >= 1.0f)
                pitch = copysignf(M_PI / 2.0f, sinp); // use 90 degrees if out of range
            else
                pitch = asinf(sinp);
            
            // yaw (z-axis rotation)
            float siny_cosp = 2.0f * (q.q0 * q.q3 + q.q1 * q.q2);
            float cosy_cosp = 1.0f - 2.0f * (q.q2 * q.q2 + q.q3 * q.q3);
            yaw = atan2f(siny_cosp, cosy_cosp);
            
            if (hall_sensor_enabled) {
                // Check hall sensor queue (non-blocking)
                hall_event_t hall_event;
                QueueHandle_t hall_queue = hall_index_get_queue();
                if (hall_queue != NULL && xQueueReceive(hall_queue, &hall_event, 0) == pdTRUE) {
                    // Reset yaw reference when hall sensor is triggered
                    global_telemetry.last_hall_pulse_us = hall_event.timestamp;
                    ESP_LOGI(TAG, "Hall sensor triggered at %" PRIu64 " us", hall_event.timestamp);
                }
            }
            
            if (motors_enabled) {
                // Calculate motor commands based on orientation
                float motor_commands[2] = {
                    -roll * 5.0f,  // Simple P controller for roll
                    -pitch * 5.0f  // Simple P controller for pitch
                };
                
                // Clamp motor commands to safe range [-1.0, 1.0]
                if (motor_commands[0] > 1.0f) motor_commands[0] = 1.0f;
                if (motor_commands[0] < -1.0f) motor_commands[0] = -1.0f;
                if (motor_commands[1] > 1.0f) motor_commands[1] = 1.0f;
                if (motor_commands[1] < -1.0f) motor_commands[1] = -1.0f;
                
                // Send commands to motor task
                xQueueOverwrite(control_queue, motor_commands);
                
                // Update telemetry
                telemetry_data_t telemetry;
                telemetry.roll = roll * 180.0f / M_PI;  // Convert to degrees
                telemetry.pitch = pitch * 180.0f / M_PI;
                telemetry.yaw = yaw * 180.0f / M_PI;
                telemetry.motor1_speed = motor_commands[0];
                telemetry.motor2_speed = motor_commands[1];
                telemetry.last_hall_pulse_us = global_telemetry.last_hall_pulse_us; // Preserve hall timestamp
                // Update global telemetry
                controller_update_telemetry(&telemetry);
            }
        } else {
            // No new IMU data, yield to avoid WDT starvation
            vTaskDelay(1);
        }
        
        // Send telemetry periodically
        static uint32_t telemetry_counter = 0;
        if (++telemetry_counter >= 20) {  // Every 20 iterations (10Hz)
            telemetry_counter = 0;
            
            // Get latest telemetry
            telemetry_data_t telemetry;
            controller_get_telemetry(&telemetry);
            
            // Call telemetry callback if registered
            if (telemetry_callback != NULL) {
                telemetry_callback(&telemetry);
            }
        }
        
        // Safety check: ensure we're not passing a zero or negative time increment
        // which would cause xTaskDelayUntil to assert and crash
        TickType_t current_tick = xTaskGetTickCount();
        if (current_tick >= last_wake_time + period) {
            // Too much time has passed, reset the wake time
            last_wake_time = current_tick;
            // Use regular delay as a fallback
            vTaskDelay(period);
        } else {
            // Normal case: wake up at regular intervals
            vTaskDelayUntil(&last_wake_time, period);
        }
    }
}

// Motor task - applies motor commands
static void motor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Motor task started");
    
    while (1) {
        // Get latest motor commands
        float motor_commands[2];
        if (xQueueReceive(control_queue, motor_commands, portMAX_DELAY) == pdTRUE) {
            if (motors_enabled) {
                // Apply motor commands (will be implemented with actual motor driver)
                // For now, just log at debug level
                ESP_LOGD(TAG, "Motor commands: %.2f, %.2f", motor_commands[0], motor_commands[1]);
                
                // When motor driver is implemented, handle errors like this:
                /*
                esp_err_t ret = motor_driver_set_speed(motor_commands[0], motor_commands[1]);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to set motor speed: %s", esp_err_to_name(ret));
                    // Log error but don't store in telemetry
                } else {
                    // Log motor driver recovery if needed
                    ESP_LOGI(TAG, "Motor driver communication working");
                }
                */
            }
        }
    }
}


// Helper function to initialize I2C
esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000  // 400KHz
    };
    
    i2c_param_config(I2C_NUM_0, &conf);
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}
