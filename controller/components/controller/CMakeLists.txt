idf_component_register(
    SRCS "controller.c"
    INCLUDE_DIRS "include"
    REQUIRES "mpu6050_driver" "hall_index" "pid" "motor_drv" "esp_timer" "freertos" "common"
)
