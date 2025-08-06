# Hall-Index Sensor Component

## Overview
This component handles the Hall-effect sensor for yaw reference detection, providing interrupt-based timestamp recording.

## Features
- Interrupt handler for Hall sensor pulses
- Timestamp queue for event processing
- Configurable GPIO pin

## API Reference

### Initialization
```c
esp_err_t hall_index_init(int gpio_pin);
```

### Event Queue Access
```c
QueueHandle_t hall_index_get_queue(void);
```

## Implementation Details
- Uses IRAM_ATTR for interrupt handler to ensure real-time response
- Creates queue for timestamp events
- Connects to control task for yaw resets

## Configuration
- Default GPIO pin configurable via menuconfig
