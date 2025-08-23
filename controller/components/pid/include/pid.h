#pragma once

#include <stdbool.h>

/**
 * Simple PID controller structure.
 * Time step (dt) is assumed 1.0 per update call; tune gains accordingly or
 * scale gains externally if using a fixed control period.
 */
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;
    float out_limit;   // Symmetric limit (+/-) for output and integral anti-windup

    bool first;
} pid_controller_t;

/**
 * Initialize PID controller.
 *
 * @param pid        Controller instance
 * @param kp,ki,kd   Gains
 * @param out_limit  Symmetric absolute output limit (also used for integral clamp). If <=0, no limiting.
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_limit);

/**
 * Update (retune) gains at runtime.
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * Reset dynamic state (integral, prev_error).
 */
void pid_reset(pid_controller_t *pid);

/**
 * Compute control output.
 *
 * @param pid       Controller
 * @param setpoint  Desired value
 * @param measurement Current value
 * @return control output (clamped if out_limit>0)
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement);
