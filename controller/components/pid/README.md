# PID Controller Component

## Overview
This component implements a PID (Proportional-Integral-Derivative) control algorithm for sphere position control.

## Features
- Configurable PID parameters (Kp, Ki, Kd)
- Anti-windup for integral term
- Output limiting

## API Reference

### Initialization
```c
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float output_limit);
```

### Control Loop Update
```c
float pid_update(pid_controller_t *pid, float setpoint, float measurement);
```

### Parameter Adjustment
```c
void pid_set_parameters(pid_controller_t *pid, float kp, float ki, float kd);
```

## Implementation Details
- Uses standard PID algorithm with fixed time step
- Implements anti-windup to prevent integral term saturation
- Supports output limiting for safety

## Usage Example
```c
pid_controller_t roll_pid;
pid_init(&roll_pid, 1.0, 0.1, 0.05, 100.0);
float output = pid_update(&roll_pid, desired_angle, current_angle);
```
