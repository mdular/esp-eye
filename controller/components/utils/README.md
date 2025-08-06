# Utilities Component

## Overview
This component provides common utility functions for the sphere controller, including WiFi connectivity.

## Features
- WiFi connection management
- Configuration storage
- Common utilities and helper functions

## API Reference

### WiFi Connection
```c
esp_err_t wifi_connect_init(void);
esp_err_t wifi_connect_start(void);
esp_err_t wifi_disconnect(void);
```

### Status Information
```c
bool wifi_is_connected(void);
char* wifi_get_ip_address(void);
```

## Implementation Details
- Handles WiFi station mode connection
- Manages reconnection attempts
- Provides status information for web interface
- Stores credentials securely

## Configuration
- WiFi credentials configurable via menuconfig or config file
- Supports multiple connection profiles
