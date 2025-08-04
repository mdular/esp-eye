# ESP-Eye Project Refactoring Plan

> **Date:** 2025-08-03  
> **Purpose:** Align existing implementation with architectural vision in AI guidelines  
> **Status:** Draft for review

## 1. Current Project Assessment

### 1.1 Project Structure

Current implementation has basic structure with:
- HTTP server component
- WiFi connection management
- Simple counter loop in main.c
- WebSocket data updates

**Main gaps vs. documented architecture:**
- No component-based structure as described in architecture.md
- Missing dedicated tasks for IMU, control, motor
- No Hall-effect sensor implementation
- Basic SPIFFS for serving web content, but no comprehensive web UI

## 2. Refactoring Goals

Based on the [AI guidelines](/docs/rules/ai_guidelines.md) and [architecture document](/docs/controller/architecture.md), propose the following refactoring stages to align implementation with vision:

### 2.1 Component Structure Refactor

1. **Create component folder structure:**
   ```
   /controller
     /components
       /mpu6050_driver
       /hall_index
       /pid
       /controller
       /motor_drv
       /websocket_srv
       /utils
   ```

2. **Migrate existing code into components:**
   - Move HTTP server code to websocket_srv component
   - Move WiFi connection to utils component
   - Preserve existing functionality while restructuring

### 2.2 Task Structure Implementation

1. **Implement task structure per architecture doc:**
   - imu_task (Core 0, 200Hz)
   - control_task (Core 0, 200Hz)
   - motor_task (Core 0, event-driven)
   - web_task (Core 1, async)
   - logging_task (Core 0, 50Hz)

2. **Apply proper RTOS patterns:**
   - Use static allocation for queues
   - Use proper error handling with ESP_ERROR_CHECK
   - Implement proper task pinning to cores

### 2.3 Sensor Integration

1. **MPU6050 Driver:**
   - Implement driver based on Jeff Rowberg I2Cdev reference
   - Add quaternion fusion (Madgwick filter)

2. **Hall-Index Sensor:**
   - Implement interrupt handler with IRAM_ATTR
   - Create queue for timestamp events
   - Connect to control task for yaw resets

### 2.4 Motor Control

1. **H-Bridge Interface:**
   - Create generic motor driver component 
   - Implement PWM control for motor outputs
   - Add safety limits and emergency stop function

### 2.5 Web Interface Enhancements

1. **WebSocket Protocol:**
   - Define proper JSON command schema
   - Implement telemetry streaming (IMU data at 10Hz)
   - Add status reporting

2. **Web UI:**
   - Enhance HTML/JS interface for sphere visualization
   - Add controls for manual positioning and calibration

## 3. Coding Standard Adoption

Apply coding standards from AI guidelines:
- Use `#pragma once` in all headers
- Wrap ESP-IDF calls with `ESP_ERROR_CHECK`
- Use static allocation for RTOS objects where possible
- Mark ISRs with `IRAM_ATTR`
- Keep lookup tables in flash with `RODATA_ATTR`
- Use proper naming conventions (snake_case)
- Add Doxygen comments to all public APIs

## 4. Implementation Plan

### Phase 1: Structure Reorganization
- Create component structure
- Move existing code to components
- Ensure build system (CMakeLists.txt) reflects new structure
- Maintain existing functionality

### Phase 2: Task Implementation
- Implement task structure
- Add proper inter-task communication 
- Set up core pinning

### Phase 3: Sensor & Control Integration
- Add MPU6050 driver
- Implement Hall sensor driver
- Create control loops for self-leveling

### Phase 4: Web Interface Enhancement
- Improve WebSocket protocol
- Enhance web UI for visualization and control

## 5. Testing Approach

For each phase:
1. **Unit Testing:**
   - Create test cases for components using Unity framework
   - Use mocks for hardware dependencies

2. **Integration Testing:**
   - Hardware-in-loop tests for sensor fusion
   - End-to-end tests for web interface

3. **Performance Testing:**
   - Measure task timing and jitter
   - Verify real-time performance meets requirements

## 6. Documentation Updates

Throughout implementation:
- Update component READMEs with public API descriptions
- Add/update ADRs for significant design decisions
- Keep AI guidelines in sync with implementation

## 7. Questions for Review

1. **Hardware Availability:**
   - Is the MPU6050 hardware available for testing?
   - Are the motor drivers identified and specifications available?

2. **Priority Order:**
   - Should we prioritize sensor fusion first or motor control?
   - Is web interface enhancement more important than hardware control?

3. **Testing Environment:**
   - Do we have a physical test setup for the sphere?
   - How do we handle testing without the complete mechanical assembly?

## 8. Next Steps

1. Review and approve this refactoring plan
2. Set up CI/CD pipeline for the new component structure
3. Begin Phase 1 implementation
4. Schedule weekly reviews to track progress against plan
