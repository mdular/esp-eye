# Main Controller Component

## Overview
This component is the central control system for the sphere, coordinating sensor input, control algorithms, and motor outputs.

## Features
- Sphere self-leveling control
- Yaw centering with Hall sensor
- Task coordination
- State management

## API Reference

### Initialization
```c
esp_err_t controller_init(void);
```

### Task Creation
```c
esp_err_t controller_start_tasks(void);
```

### Status and Telemetry
```c
controller_status_t controller_get_status(void);
quaternion_t controller_get_orientation(void);
```

## Implementation Details
- Creates and manages multiple FreeRTOS tasks
- Implements real-time control loop at 200Hz
- Uses static allocation for RTOS objects
- Handles inter-task communication via queues

## Task Structure
- imu_task: Reads MPU6050 at 200Hz
- control_task: Processes orientation and computes control outputs
- motor_task: Drives motors based on control commands
