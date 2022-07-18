idf_component_register(
    SRCS "bme680.c"
    INCLUDE_DIRS .
    REQUIRES data_manager i2cdev log esp_idf_lib_helpers sensor_handler algobsec iot_time
)
