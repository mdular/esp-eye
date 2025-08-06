# ESP-Eye Sphere Controller Components

This directory contains the modular components for the ESP-Eye sphere controller project, organized according to the architecture document.

## Component Structure

### Core Components

- **mpu6050_driver**: MPU6050 IMU driver with quaternion fusion
- **hall_index**: Hall-effect sensor interface for yaw reference
- **pid**: PID controller implementation for position control
- **controller**: Main controller logic and task coordination
- **motor_drv**: H-bridge motor driver interface

### Support Components

- **websocket_srv**: WebSocket server for telemetry and commands
- **utils**: Utility functions, including WiFi connection management

## Task Structure

The system is organized into the following FreeRTOS tasks:

1. **imu_task**: Reads IMU data at 200Hz, Core 0
2. **control_task**: Processes orientation and calculates motor commands, Core 0
3. **motor_task**: Handles motor control, Core 0
4. **web_task**: Manages WebSocket communication, Core 1
5. **logging_task**: System logging, Core 0

## Coding Standards

All components follow these standards:

- `#pragma once` in all headers
- `ESP_ERROR_CHECK` for ESP-IDF calls
- Static allocation for RTOS objects
- `IRAM_ATTR` for ISRs
- Snake case naming convention
- Doxygen comments for all public APIs

## Build System

Each component has its own CMakeLists.txt with proper dependencies declared.
