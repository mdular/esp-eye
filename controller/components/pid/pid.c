#include <stdio.h>
#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float output_limit) {
    if (pid == NULL) {
        return;
    }
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = output_limit;
    pid->dt = 0.005f;  // Default 5ms (200Hz)
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement) {
    if (pid == NULL) {
        return 0.0f;
    }
    
    pid->setpoint = setpoint;
    
    // Calculate error
    float error = setpoint - measurement;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * pid->dt;
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / pid->dt;
    float d_term = pid->kd * derivative;
    
    // Save error for next iteration
    pid->prev_error = error;
    
    // Calculate output
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}

void pid_set_parameters(pid_controller_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
