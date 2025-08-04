#pragma once

/**
 * @brief PID controller structure
 */
typedef struct {
    float kp;             // Proportional gain
    float ki;             // Integral gain
    float kd;             // Derivative gain
    float setpoint;       // Target value
    float integral;       // Integral term
    float prev_error;     // Previous error for derivative
    float output_limit;   // Output limit
    float dt;             // Time step in seconds
} pid_controller_t;

/**
 * @brief Initialize a PID controller
 * 
 * @param pid Pointer to PID controller structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param output_limit Maximum output value (symmetrical +/-)
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float output_limit);

/**
 * @brief Update PID controller with new measurement
 * 
 * @param pid Pointer to PID controller structure
 * @param setpoint Target value
 * @param measurement Current measured value
 * @return float Control output
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement);

/**
 * @brief Set new PID parameters
 * 
 * @param pid Pointer to PID controller structure
 * @param kp New proportional gain
 * @param ki New integral gain
 * @param kd New derivative gain
 */
void pid_set_parameters(pid_controller_t *pid, float kp, float ki, float kd);
