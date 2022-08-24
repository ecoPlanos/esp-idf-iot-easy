idf_component_register(
    SRCS virtual_sensor.c
    INCLUDE_DIRS .
    REQUIRES log esp_idf_lib_helpers sensor_handler
)
