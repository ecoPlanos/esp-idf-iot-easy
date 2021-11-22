idf_component_register(
    SRCS switch_sen.c
    INCLUDE_DIRS .
    REQUIRES data_manager log esp_idf_lib_helpers sensor_handler
)
