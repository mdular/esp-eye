idf_component_register(
    SRCS "motor_drv.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "esp_timer"
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
