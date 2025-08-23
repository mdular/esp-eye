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
#include "pid.h"
#include "motor_drv.h"
#include <math.h>

static const char *TAG = "controller";

// Angle normalization helper (-pi, pi]
static inline float normalize_angle(float a) {
    while (a > M_PI)  a -= 2.0f * M_PI;
    while (a <= -M_PI) a += 2.0f * M_PI;
    return a;
}

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
static QueueHandle_t control_queue = NULL;
static QueueHandle_t orientation_queue = NULL;

 // Static queue buffers and storage areas (single-element latest-only queues)

static StaticQueue_t control_queue_buffer;
static uint8_t control_queue_storage[sizeof(float) * 2];

static StaticQueue_t orientation_queue_buffer;
static uint8_t orientation_queue_storage[sizeof(orientation_sample_t)];

// Telemetry callback
static controller_telemetry_cb_t telemetry_callback = NULL;

// Global telemetry data storage
static telemetry_data_t global_telemetry = {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .last_hall_pulse_us = 0,
    .hall_timeout_count = 0,
    .motor1_speed = 0.0f,
    .motor2_speed = 0.0f,
    .motor_sat_count = 0,
    .ctrl_q_depth = 0,
    .orient_q_depth = 0,
    .imu_missed_reads = 0,
    .imu_sequence = 0,
    .status = 0,
    .error_message = {0}
};

// PID controllers (initialized in control_task)
// Removed roll PID (mechanical system is 2-DOF: elevation=pitch, azimuth=yaw)
static pid_controller_t pid_pitch;
static pid_controller_t pid_yaw;  // yaw PID (uses unwrapped yaw accumulator)

// Yaw zeroing offset (radians) set on hall index pulse
static volatile float yaw_zero_offset = 0.0f;
// Last raw yaw sample (radians) captured in imu_task for hall pulse alignment
static volatile float last_yaw_sample = 0.0f;

// Continuous unwrapped yaw accumulator (radians) reset on hall index pulse
static volatile float yaw_unwrapped = 0.0f;
static volatile float prev_yaw_raw = 0.0f;
static volatile bool yaw_unwrap_initialized = false;

// Yaw setpoint (radians) placeholder for future command integration
static volatile float yaw_setpoint = 0.0f;

// Task prototypes
static void imu_task(void *pvParameters);
static void control_task(void *pvParameters);
static void motor_task(void *pvParameters);

// Helper function prototypes
esp_err_t i2c_master_init(void);

esp_err_t controller_init(void) {
    // Create queues (static, latest-only)
    control_queue = xQueueCreateStatic(1, sizeof(float) * 2, control_queue_storage, &control_queue_buffer);
    if (control_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create control_queue");
        return ESP_FAIL;
    }

    orientation_queue = xQueueCreateStatic(1, sizeof(orientation_sample_t), orientation_queue_storage, &orientation_queue_buffer);
    if (orientation_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create orientation_queue");
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

/**
 * @brief Emergency stop implementation
 *
 * Disables further motor updates, issues hardware stop, zeroes motor telemetry,
 * and sets status=2 (ESTOP) for UI diagnostics.
 */
esp_err_t controller_emergency_stop(void) {
    // Disable motors so control loop stops sending new commands
    controller_set_motors_enabled(false);
    // Attempt hardware emergency stop (ignore failure to continue state update)
    motor_drv_emergency_stop();
    // Update telemetry
    telemetry_data_t t;
    controller_get_telemetry(&t);
    t.motor1_speed = 0.0f;
    t.motor2_speed = 0.0f;
    t.status = 2; // ESTOP
    controller_update_telemetry(&t);
    ESP_LOGW(TAG, "Emergency stop invoked");
    return ESP_OK;
}

/**
 * @brief Set yaw setpoint in radians (unwrapped frame).
 */
void controller_set_yaw_setpoint(float yaw_rad) {
    yaw_setpoint = yaw_rad;
}

/**
 * @brief Get current yaw setpoint in radians.
 */
float controller_get_yaw_setpoint(void) {
    return yaw_setpoint;
}

// IMU task - samples IMU at 200Hz, runs Madgwick, publishes latest orientation
static void imu_task(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "IMU task started");

    const TickType_t period = pdMS_TO_TICKS(5); // 200Hz
    TickType_t last_wake = xTaskGetTickCount();

    static uint32_t seq = 0;
    static uint32_t missed_reads = 0;

    mpu_data_t raw;

    while (1) {
        // Sample sensor
        esp_err_t ret = mpu6050_get_raw(I2C_NUM_0, &raw);
        if (ret != ESP_OK) {
            missed_reads++;
            // Throttle logging to avoid spam
            if ((missed_reads & 0xFF) == 0) {
                ESP_LOGW(TAG, "IMU read failures: %u (last err=%s)", missed_reads, esp_err_to_name(ret));
            }
        } else {
            // Run fusion
            quaternion_t q = madgwick_update(&raw);

            // Convert quaternion to Euler (radians)
            float q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
            float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
            float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
            float roll = atan2f(sinr_cosp, cosr_cosp);

            float sinp = 2.0f * (q0 * q2 - q3 * q1);
            float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(M_PI / 2.0f, sinp) : asinf(sinp);

            float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
            float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
            float yaw = atan2f(siny_cosp, cosy_cosp);

            // Continuous yaw unwrapping (accumulator) independent of hall zeroing
            if (!yaw_unwrap_initialized) {
                prev_yaw_raw = yaw;
                yaw_unwrapped = 0.0f;
                yaw_unwrap_initialized = true;
            } else {
                float delta = yaw - prev_yaw_raw;
                if (delta > M_PI)       delta -= 2.0f * M_PI;
                else if (delta < -M_PI) delta += 2.0f * M_PI;
                yaw_unwrapped += delta;
                prev_yaw_raw = yaw;
            }

            orientation_sample_t sample = {
                .q = q,
                .roll = roll,
                .pitch = pitch,
                .yaw = yaw,
                .sample_time_us = esp_timer_get_time(),
                .seq = ++seq,
                .missed_reads = missed_reads
            };
            xQueueOverwrite(orientation_queue, &sample);

            // Record last raw yaw (radians) for hall index zeroing
            last_yaw_sample = yaw;

            // Apply yaw zero offset (normalized) and update global telemetry (degrees for UI)
            float yaw_corr = normalize_angle(yaw - yaw_zero_offset);

            telemetry_data_t t = global_telemetry; // copy current
            t.q0 = q0; t.q1 = q1; t.q2 = q2; t.q3 = q3;
            t.roll = roll * 180.0f / M_PI;
            t.pitch = pitch * 180.0f / M_PI;
            t.yaw = yaw_corr * 180.0f / M_PI;
            t.imu_missed_reads = missed_reads;
            t.imu_sequence = seq;
            t.status = 1; // running
            controller_update_telemetry(&t);
        }

        // Periodic wait
        vTaskDelayUntil(&last_wake, period);
    }
}

// Control task - consumes orientation samples and produces motor commands
static void control_task(void *pvParameters) {
    (void)pvParameters;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5); // 200Hz

    ESP_LOGI(TAG, "Control task started");

    // Initialize PID controllers (provisional gains; to be tuned)
    static bool pid_inited = false;
    if (!pid_inited) {
        pid_init(&pid_pitch, 0.5f, 0.0f, 0.0f, 1.0f); // elevation
        pid_init(&pid_yaw,   0.2f, 0.0f, 0.0f, 1.0f); // azimuth (placeholder gains)
        pid_inited = true;
    }

    // Hall timeout tracking
    static bool hall_timeout_active = false;
    const int64_t HALL_TIMEOUT_US = 1000000; // 1s without pulse triggers timeout count

    // Allow system to stabilize
    vTaskDelay(period);
    last_wake_time = xTaskGetTickCount();

    while (1) {
        orientation_sample_t sample;
        bool have_sample = (xQueueReceive(orientation_queue, &sample, 0) == pdTRUE);

        // Hall sensor events & timeout detection
        if (hall_sensor_enabled) {
            hall_event_t hall_event;
            QueueHandle_t hall_queue = hall_index_get_queue();
            if (hall_queue != NULL && xQueueReceive(hall_queue, &hall_event, 0) == pdTRUE) {
                global_telemetry.last_hall_pulse_us = hall_event.timestamp;
                float prev_offset = yaw_zero_offset;
                yaw_zero_offset = last_yaw_sample; // zero yaw (raw frame)
                yaw_unwrapped = 0.0f;              // reset accumulator relative to new index
                hall_timeout_active = false; // reset timeout state
                ESP_LOGI(TAG,
                         "Hall index @ %" PRIu64 " us -> yaw zeroed (prev offset=%.2f deg, new offset=%.2f deg)",
                         hall_event.timestamp,
                         prev_offset * 180.0f / M_PI,
                         yaw_zero_offset * 180.0f / M_PI);
            } else {
                if (global_telemetry.last_hall_pulse_us != 0) {
                    int64_t now = esp_timer_get_time();
                    if (!hall_timeout_active &&
                        (now - (int64_t)global_telemetry.last_hall_pulse_us) > HALL_TIMEOUT_US) {
                        // Increment hall timeout counter
                        telemetry_data_t t;
                        controller_get_telemetry(&t);
                        t.hall_timeout_count++;
                        controller_update_telemetry(&t);
                        hall_timeout_active = true;
                        ESP_LOGW(TAG, "Hall timeout (> %lld us without pulse)", (long long)HALL_TIMEOUT_US);
                    }
                    // Once pulse arrives hall_timeout_active reset above
                }
            }
        }

        if (motors_enabled && have_sample) {
            // Setpoints: pitch (elevation) -> 0 rad (level), yaw tracks yaw_setpoint (unwrapped frame)
            float pitch_output = pid_update(&pid_pitch, 0.0f, sample.pitch);
            float yaw_output   = pid_update(&pid_yaw,   yaw_setpoint, yaw_unwrapped);
            float yaw_error = yaw_setpoint - yaw_unwrapped;

            // Map axes to motors: motor0 = elevation, motor1 = azimuth
            float motor_commands[2];
            motor_commands[0] = pitch_output;
            motor_commands[1] = yaw_output;

            // Periodic debug
            static uint32_t yaw_log_counter = 0;
            if (++yaw_log_counter >= 100) { // ~0.5s @200Hz
                yaw_log_counter = 0;
                ESP_LOGD(TAG, "PitchOut=%.3f Yaw sp=%.3f cur=%.3f err=%.3f out=%.3f",
                         pitch_output, yaw_setpoint, yaw_unwrapped, yaw_error, yaw_output);
            }

            // Detect saturation & clamp
            bool saturated = false;
            for (int i = 0; i < 2; i++) {
                if (motor_commands[i] > 1.0f) { motor_commands[i] = 1.0f; saturated = true; }
                else if (motor_commands[i] < -1.0f) { motor_commands[i] = -1.0f; saturated = true; }
            }

            // Send commands (latest only)
            xQueueOverwrite(control_queue, motor_commands);

            // Update telemetry (retain other fields)
            telemetry_data_t t;
            controller_get_telemetry(&t);
            t.motor1_speed = motor_commands[0];
            t.motor2_speed = motor_commands[1];
            if (saturated) {
                t.motor_sat_count++;
            }
            // Queue depths (diagnostics)
            t.ctrl_q_depth = (uint8_t)uxQueueMessagesWaiting(control_queue);
            t.orient_q_depth = (uint8_t)uxQueueMessagesWaiting(orientation_queue);
            controller_update_telemetry(&t);
        } else {
            // Still update queue depth diagnostics periodically even if no sample
            static uint8_t idle_counter = 0;
            if (++idle_counter >= 20) { // ~100ms
                idle_counter = 0;
                telemetry_data_t t;
                controller_get_telemetry(&t);
                t.ctrl_q_depth = (uint8_t)uxQueueMessagesWaiting(control_queue);
                t.orient_q_depth = (uint8_t)uxQueueMessagesWaiting(orientation_queue);
                controller_update_telemetry(&t);
            }
            if (!have_sample) {
                vTaskDelay(1);
            }
        }

        // Periodic telemetry callback (10Hz)
        static uint32_t telemetry_counter = 0;
        if (++telemetry_counter >= 20) {
            telemetry_counter = 0;
            telemetry_data_t telemetry;
            controller_get_telemetry(&telemetry);
            controller_telemetry_cb_t cb = telemetry_callback;
            if (cb != NULL) {
                cb(&telemetry);
            }
        }

        // Maintain loop period
        TickType_t current_tick = xTaskGetTickCount();
        if (current_tick >= last_wake_time + period) {
            last_wake_time = current_tick;
            vTaskDelay(period);
        } else {
            vTaskDelayUntil(&last_wake_time, period);
        }
    }
}

// Motor task - applies motor commands
static void motor_task(void *pvParameters) {
    (void)pvParameters;
    ESP_LOGI(TAG, "Motor task started");

    static bool motor_inited = false;
    uint32_t init_fail_throttle = 0;

    while (1) {
        // Lazy init (after WiFi / other subsystems if desired)
        if (motors_enabled && !motor_inited) {
            esp_err_t r = motor_drv_init();
            if (r == ESP_OK) {
                motor_inited = true;
                ESP_LOGI(TAG, "Motor driver initialized");
            } else {
                if ((init_fail_throttle++ & 0x3F) == 0) {
                    ESP_LOGW(TAG, "Motor driver init failed: %s (will retry)", esp_err_to_name(r));
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        // Get latest motor commands
        float motor_commands[2];
        if (xQueueReceive(control_queue, motor_commands, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (motors_enabled && motor_inited) {
                ESP_LOGD(TAG, "Motor commands: %.2f, %.2f", motor_commands[0], motor_commands[1]);
                esp_err_t r0 = motor_drv_set_speed(0, motor_commands[0]);
                esp_err_t r1 = motor_drv_set_speed(1, motor_commands[1]);
                if (r0 != ESP_OK || r1 != ESP_OK) {
                    ESP_LOGW(TAG, "motor_drv_set_speed error (m0=%s m1=%s)",
                             (r0==ESP_OK?"OK":esp_err_to_name(r0)),
                             (r1==ESP_OK?"OK":esp_err_to_name(r1)));
                }
            }
        } else {
            // No new command; could implement timeout-based decay or watchdog kick here
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
