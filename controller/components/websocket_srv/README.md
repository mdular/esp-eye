# WebSocket Server Component

## Overview
This component provides the web interface for sphere control and telemetry, including HTTP server and WebSocket communication.

## Features
- HTTP server for static web content
- WebSocket server for real-time communication
- JSON command protocol
- Telemetry streaming at 10Hz

## API Reference

### Initialization
```c
esp_err_t websocket_srv_init(void);
```

### Task Creation
```c
esp_err_t websocket_srv_start(void);
```

### Data Publishing
```c
esp_err_t websocket_srv_publish_telemetry(const telemetry_data_t *data);
```

## Implementation Details
- Uses ESP-IDF HTTP server component
- Implements WebSocket protocol for bidirectional communication
- Serves static files from SPIFFS
- Runs on Core 1 to avoid interference with control tasks

## Protocol
- JSON-based command and telemetry protocol
- Supports control commands, calibration, and configuration
- Streams orientation data at configurable rate
