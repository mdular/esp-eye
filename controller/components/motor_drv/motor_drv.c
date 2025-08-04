#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
#include "motor_drv.h"

static const char *TAG = "motor_drv";

// PWM configuration
#define MOTOR_PWM_FREQ 1000  // 1kHz
#define MOTOR_PWM_RESOLUTION MCPWM_TIMER_RESOLUTION_NANOSEC
#define MOTOR_PWM_PERIOD (1000000000 / MOTOR_PWM_FREQ)  // Period in nanoseconds

// Motor GPIO pins (to be configured via menuconfig in the future)
#define MOTOR1_PWM_A_PIN 32
#define MOTOR1_PWM_B_PIN 33
#define MOTOR2_PWM_A_PIN 25
#define MOTOR2_PWM_B_PIN 26

esp_err_t motor_drv_init(void) {
    ESP_LOGI(TAG, "Initializing motor driver...");
    
    // Initialize MCPWM for motor 1
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1_PWM_A_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR1_PWM_B_PIN);
    
    // Initialize MCPWM for motor 2
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR2_PWM_A_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR2_PWM_B_PIN);
    
    // Configure MCPWM timer 0
    mcpwm_config_t pwm_config = {
        .frequency = MOTOR_PWM_FREQ,
        .cmpr_a = 0,  // Start with 0% duty cycle
        .cmpr_b = 0,  // Start with 0% duty cycle
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };
    
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config));
    
    // Start with motors stopped
    ESP_ERROR_CHECK(motor_drv_emergency_stop());
    
    ESP_LOGI(TAG, "Motor driver initialized successfully");
    return ESP_OK;
}

esp_err_t motor_drv_set_speed(int motor_id, float speed) {
    // Validate input
    if (motor_id < 0 || motor_id > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) {
        speed = 1.0f;
    } else if (speed < -1.0f) {
        speed = -1.0f;
    }
    
    // Convert speed to duty cycle (0-100%)
    float duty_cycle = fabsf(speed) * 100.0f;
    
    if (motor_id == 0) {
        if (speed >= 0) {
            // Forward
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
        } else {
            // Reverse
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);
        }
    } else {
        if (speed >= 0) {
            // Forward
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
        } else {
            // Reverse
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty_cycle);
        }
    }
    
    return ESP_OK;
}

esp_err_t motor_drv_emergency_stop(void) {
    // Set all PWM outputs to 0
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    
    ESP_LOGW(TAG, "Emergency stop activated");
    return ESP_OK;
}
