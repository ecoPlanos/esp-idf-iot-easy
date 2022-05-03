idf_component_register(
    SRCS senseair_k30.c
    INCLUDE_DIRS .
    REQUIRES data_manager i2cdev log esp_idf_lib_helpers sensor_handler
)
