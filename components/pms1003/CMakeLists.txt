idf_component_register(
    SRCS pms1003.c
    INCLUDE_DIRS .
    REQUIRES data_manager log esp_idf_lib_helpers sensor_handler uartdev
)
