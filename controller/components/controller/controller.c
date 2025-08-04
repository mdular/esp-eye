#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "controller.h"
#include <math.h>

static const char *TAG = "controller";

// Task parameters
#define IMU_TASK_STACK_SIZE 3072
#define CONTROL_TASK_STACK_SIZE 3072
#define MOTOR_TASK_STACK_SIZE 3072
#define LOGGING_TASK_STACK_SIZE 2048

#define IMU_TASK_PRIORITY 6
#define CONTROL_TASK_PRIORITY 6
#define MOTOR_TASK_PRIORITY 5
#define LOGGING_TASK_PRIORITY 2

// Telemetry callback
static controller_telemetry_cb_t telemetry_callback = NULL;

// Simulated data for initial implementation
static quaternion_t current_orientation = {1.0f, 0.0f, 0.0f, 0.0f};  // Identity quaternion
static controller_status_t current_status = CONTROLLER_IDLE;
static int counter = 0;  // Simple counter for testing

// Task handles
static TaskHandle_t imu_task_handle = NULL;
static TaskHandle_t control_task_handle = NULL;
static TaskHandle_t motor_task_handle = NULL;
static TaskHandle_t logging_task_handle = NULL;

// Inter-task communication
static QueueHandle_t imu_queue = NULL;
static QueueHandle_t control_queue = NULL;
static QueueHandle_t logging_queue = NULL;

// Forward declarations
static void imu_task(void *pvParameters);
static void control_task(void *pvParameters);
static void motor_task(void *pvParameters);
static void logging_task(void *pvParameters);

esp_err_t controller_init(void) {
    // Create queues for inter-task communication
    imu_queue = xQueueCreate(1, sizeof(mpu_data_t));
    if (imu_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create imu_queue");
        return ESP_FAIL;
    }
    
    control_queue = xQueueCreate(1, sizeof(float) * 2);  // For motor commands
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control_queue");
        vQueueDelete(imu_queue);
        return ESP_FAIL;
    }
    
    logging_queue = xQueueCreate(10, sizeof(char *));  // For log messages
    if (logging_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create logging_queue");
        vQueueDelete(imu_queue);
        vQueueDelete(control_queue);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Controller initialized successfully");
    return ESP_OK;
}

esp_err_t controller_start_tasks(void) {
    BaseType_t ret;
    
    // Create IMU task
    ESP_LOGI(TAG, "Starting IMU task...");
    ret = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        &imu_task_handle,
        0  // Pin to Core 0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create imu_task");
        return ESP_FAIL;
    }
    
    // Delay to avoid power spike
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Create control task
    ESP_LOGI(TAG, "Starting control task...");
    ret = xTaskCreatePinnedToCore(
        control_task,
        "control_task",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        &control_task_handle,
        0  // Pin to Core 0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control_task");
        return ESP_FAIL;
    }
    
    // Delay to avoid power spike
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Create motor task
    ESP_LOGI(TAG, "Starting motor task...");
    ret = xTaskCreatePinnedToCore(
        motor_task,
        "motor_task",
        MOTOR_TASK_STACK_SIZE,
        NULL,
        MOTOR_TASK_PRIORITY,
        &motor_task_handle,
        0  // Pin to Core 0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor_task");
        return ESP_FAIL;
    }
    
    // Delay to avoid power spike
    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    // Create logging task
    ESP_LOGI(TAG, "Starting logging task...");
    ret = xTaskCreatePinnedToCore(
        logging_task,
        "logging_task",
        LOGGING_TASK_STACK_SIZE,
        NULL,
        LOGGING_TASK_PRIORITY,
        &logging_task_handle,
        0  // Pin to Core 0
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create logging_task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "All controller tasks started");
    current_status = CONTROLLER_RUNNING;
    return ESP_OK;
}

controller_status_t controller_get_status(void) {
    return current_status;
}

quaternion_t controller_get_orientation(void) {
    return current_orientation;
}

esp_err_t controller_get_telemetry(telemetry_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Fill in telemetry data
    data->pitch = 0.0f;  // Simulated data for now
    data->roll = 0.0f;   // Simulated data for now
    data->yaw = 0.0f;    // Simulated data for now
    data->motor1_speed = 0.0f;  // Simulated data for now
    data->motor2_speed = 0.0f;  // Simulated data for now
    data->last_hall_pulse_us = 0;  // Simulated data for now
    data->counter = counter;  // Include the counter in telemetry
    
    return ESP_OK;
}

esp_err_t controller_register_telemetry_callback(controller_telemetry_cb_t callback) {
    if (callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    telemetry_callback = callback;
    return ESP_OK;
}

// Simulated IMU task - will be replaced with actual MPU6050 driver
static void imu_task(void *pvParameters) {
    // Initialize the last wake time to the current tick count
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Ensure we have at least 1 tick period (minimum safe value)
    const TickType_t period = pdMS_TO_TICKS(10); // Use 10ms (100Hz) instead of 5ms to ensure we have enough ticks
    
    ESP_LOGI(TAG, "IMU task started");
    
    // Delay for one period before starting the loop to ensure stability
    vTaskDelay(period);
    
    // Reset the last wake time after the initial delay
    last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Simulate IMU data
        mpu_data_t imu_data = {0};
        
        // Send data to control task
        xQueueOverwrite(imu_queue, &imu_data);
        
        // Simulate orientation (rotating in yaw)
        float angle = (counter % 360) * (3.14159f / 180.0f);  // Convert to radians
        current_orientation.q0 = cosf(angle / 2.0f);
        current_orientation.q3 = sinf(angle / 2.0f);
        counter++;
        
        // Use safe vTaskDelay if xTaskDelayUntil keeps failing
        // vTaskDelay(period);
        
        // Wake up at regular intervals
        vTaskDelayUntil(&last_wake_time, period);
    }
}

// Control task - calculates motor commands
static void control_task(void *pvParameters) {
    // Initialize the last wake time to the current tick count
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Ensure we have at least 1 tick period (minimum safe value)
    const TickType_t period = pdMS_TO_TICKS(10); // Use 10ms (100Hz) instead of 5ms
    
    ESP_LOGI(TAG, "Control task started");
    
    // Delay for one period before starting the loop to ensure stability
    vTaskDelay(period);
    
    // Reset the last wake time after the initial delay
    last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Get latest IMU data
        mpu_data_t imu_data;
        if (xQueueReceive(imu_queue, &imu_data, 0) == pdTRUE) {
            // Calculate motor commands (simple simulation for now)
            float motor_commands[2] = {0.0f, 0.0f};
            
            // Send commands to motor task
            xQueueOverwrite(control_queue, motor_commands);
        }
        
        // Prepare telemetry data
        telemetry_data_t telemetry;
        controller_get_telemetry(&telemetry);
        
        // Send telemetry every 10 cycles (10Hz)
        if (counter % 10 == 0 && telemetry_callback != NULL) {
            telemetry_callback(&telemetry);
        }
        
        // Use safe vTaskDelay as a fallback if needed
        // vTaskDelay(period);
        
        // Wake up at regular intervals
        vTaskDelayUntil(&last_wake_time, period);
    }
}

// Motor task - applies motor commands
static void motor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Motor task started");
    
    while (1) {
        // Get latest motor commands
        float motor_commands[2];
        if (xQueueReceive(control_queue, motor_commands, portMAX_DELAY) == pdTRUE) {
            // Apply motor commands (will be implemented with actual motor driver)
            // For now, just log
            ESP_LOGD(TAG, "Motor commands: %.2f, %.2f", motor_commands[0], motor_commands[1]);
        }
    }
}

// Logging task - handles debug messages
static void logging_task(void *pvParameters) {
    // Initialize the last wake time to the current tick count
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Ensure we have at least 1 tick period (minimum safe value)
    const TickType_t period = pdMS_TO_TICKS(20);  // 50Hz
    
    ESP_LOGI(TAG, "Logging task started");
    
    // Delay for one period before starting the loop to ensure stability
    vTaskDelay(period);
    
    // Reset the last wake time after the initial delay
    last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // Process log messages
        char *log_message;
        while (xQueueReceive(logging_queue, &log_message, 0) == pdTRUE) {
            printf("%s\n", log_message);
            free(log_message);  // Free dynamically allocated message
        }
        
        // Periodic status update - less frequent to reduce power consumption
        if (counter % 200 == 0) {  // Every ~4 seconds at 50Hz
            ESP_LOGI(TAG, "System running. Counter: %d", counter);
        }
        
        // Use safe vTaskDelay as a fallback if needed
        // vTaskDelay(period);
        
        // Wake up at regular intervals
        vTaskDelayUntil(&last_wake_time, period);
    }
}
