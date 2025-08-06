# MPU6050 Driver Component

## Overview
This component provides a driver for the MPU6050 IMU sensor, implementing I2C communication and sensor data processing.

## Features
- Initialization of MPU6050 sensor over I2C
- Raw acceleration and gyroscope data reading
- Quaternion fusion using Madgwick filter
- 200Hz sample rate support

## API Reference

### Initialization
```c
esp_err_t mpu6050_init(i2c_dev_t *dev);
```

### Data Reading
```c
esp_err_t mpu6050_get_raw(i2c_dev_t *dev, mpu_data_t *data);
```

### Quaternion Calculation
```c
quaternion_t madgwick_update(mpu_data_t raw);
```

## Configuration
- Default I2C address: 0x68
- Supports custom I2C port and pins via menuconfig

## Dependencies
- ESP-IDF I2C driver
- Requires I2C bus initialization before use
