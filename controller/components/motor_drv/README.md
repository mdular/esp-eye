# Motor Driver Component

## Overview
This component provides control for the H-bridge motor driver circuit that controls the sphere motors.

## Features
- PWM control for motor outputs
- Safety limits and emergency stop
- Support for different motor configurations

## API Reference

### Initialization
```c
esp_err_t motor_drv_init(void);
```

### Motor Control
```c
esp_err_t motor_drv_set_speed(int motor_id, float speed);
esp_err_t motor_drv_emergency_stop(void);
```

## Implementation Details
- Uses ESP32 MCPWM peripheral for accurate PWM generation
- Implements safety limits to prevent hardware damage
- Supports configurable motor parameters

## Configuration
- PWM frequency: 1kHz (default)
- Motor output pins configurable via menuconfig
- Deadtime protection for H-bridge switching
