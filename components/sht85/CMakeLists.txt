idf_component_register(
    SRCS sht85.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers sensor_handler
)
