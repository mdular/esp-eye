#include "pid.h"

static float clamp_sym(float v, float limit) {
    if (limit <= 0.0f) return v;
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_limit) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_limit = out_limit;
    pid->first = true;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_reset(pid_controller_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first = true;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement) {
    if (!pid) return 0.0f;

    float error = setpoint - measurement;

    // Proportional
    float P = pid->kp * error;

    // Integral with anti-windup via clamping
    pid->integral += pid->ki * error;
    pid->integral = clamp_sym(pid->integral, pid->out_limit);

    // Derivative (on measurement difference) using error difference; ignore derivative first cycle
    float derivative = 0.0f;
    if (!pid->first) {
        float d_error = (error - pid->prev_error);
        derivative = pid->kd * d_error;
    } else {
        pid->first = false;
    }

    pid->prev_error = error;

    float output = P + pid->integral + derivative;
    output = clamp_sym(output, pid->out_limit);
    return output;
}
