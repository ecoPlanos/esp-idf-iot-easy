/*
 * Copyright (c) 2021 Ecoplanos: Automação e Sistemas Integrados, Lda.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SWITCH_SEN_H__
#define __SWITCH_SEN_H__

#include <driver/gpio.h>
#include <esp_err.h>
#include <sensor_handler.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SWITCH_SEN_LIB_VERSION_MAJOR 1
#define SWITCH_SEN_LIB_VERSION_MINOR 0
#define SWITCH_SEN_LIB_VERSION_PATCH 0
#define SWITCH_SEN_LIB_VERSION  (SWITCH_SEN_LIB_VERSION_MAJOR << 16)|(SWITCH_SEN_LIB_VERSION_MINOR << 8)|SWITCH_SEN_LIB_VERSION_PATCH
// #define LIB_VERSION_SWITCH_SEN 1.0

#define SWITCH_SEN_CONF_FILE_NAME "conf.cfg"
#define SWITCH_SEN_DATA_FILE_NAME "dat.txt"
#define SWITCH_SEN_FILE_DIR "swsen"
#define SWITCH_SEN_CONF_FILE_PATH SWITCH_SEN_FILE_DIR "/" SWITCH_SEN_CONF_FILE_NAME
#define SWITCH_SEN_DATA_FILE_PATH SWITCH_SEN_FILE_DIR "/" SWITCH_SEN_DATA_FILE_NAME

/**
 * Sensor configuration
 */
typedef struct {
  sen_out_trig_dir_type_t trig_dir; //trigger direction
  uint32_t min_period_us;           //debounce period
  gpio_num_t gpio;
  uint32_t ver;                     //configuration version
} switch_sen_conf_t;

/**
 * Device info.
 */
typedef struct {
    uint8_t pack_id;    // Package Identification
    uint8_t dev_id;     // Device Identification
    // uint8_t status;
} switch_sen_inf_t;

/**
 * Switch ensor device data structure type
 */
typedef struct {
    switch_sen_conf_t conf; //Sensor configuration
    switch_sen_inf_t info;  //Sensor information
    sensor_t sen;
} switch_sen_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t switch_sen_init(switch_sen_t *dev, sen_out_trig_dir_type_t trigger_dir, uint32_t min_period_us, gpio_num_t input_pin, gpio_pullup_t pull_up_en,gpio_pulldown_t pull_down_en, uint8_t dev_id, uint8_t pack_id, uint8_t sen_id, char sen_name[]);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t switch_sen_free(switch_sen_t *dev);

esp_err_t switch_sen_get_measurement_duration(switch_sen_t *dev, uint32_t *duration);

esp_err_t switch_sen_get_val(switch_sen_t *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SWITCH_SEN_H__ */
