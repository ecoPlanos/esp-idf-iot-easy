idf_component_register(
    SRCS analog_sen.c
    INCLUDE_DIRS .
    REQUIRES data_manager esp_adc_cal log esp_idf_lib_helpers sensor_handler
)
