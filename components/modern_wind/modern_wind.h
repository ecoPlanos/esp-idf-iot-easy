/*
 * Copyright (c) 2022 Ecoplanos: Automação e Sistemas Integrados, Lda.
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


#ifndef __MODERN_WIND_H__
#define __MODERN_WIND_H__

#include <analog_sen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODERN_WIND_LIB_VERSION_MAJOR 1
#define MODERN_WIND_LIB_VERSION_MINOR 0
#define MODERN_WIND_LIB_VERSION_PATCH 0
#define MODERN_WIND_LIB_VERSION  (MODERN_WIND_LIB_VERSION_MAJOR << 16)|(MODERN_WIND_LIB_VERSION_MINOR << 8)|MODERN_WIND_LIB_VERSION_PATCH

#define MODERN_WIND_ZERO_WIND_MV 1369.2
#define MODERN_WIND_TEMPERATURE_TC 19.5
#define MODERN_WIND_TEMPERATURE_V0 400

enum {
  MODERN_WIND_WIND_ID = 0,
  MODERN_WIND_TEMP_ID
};

typedef struct {
  float zero_wind_mv; // zero wind voltage
  float tc;  // temperature coefficient
  float v0;  // the voltage at zero degrees C
}modern_wind_conf_t;
/**
 * @brief Initialize device
 *
 * @param modern_wind_sen Analog sensor descriptor
 * @return `ESP_OK` on success
 */
esp_err_t modern_wind_init(analog_sen_t *modern_wind_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t wind_unit, adc_channel_t wind_channel, adc_unit_t tmp_unit, adc_channel_t tmp_channel, gpio_num_t shdn_gpio, char *sen_name);

/**
* @brief Free device descriptor
*
* @param modern_wind_sen Analog sensor descriptor
* @param oud_idx Analog output index
* @param unit ADC unit
* @param analog_channel ADC channel
* @return `ESP_OK` on success
*/
esp_err_t modern_wind_config_output(analog_sen_t *modern_wind_sen, uint8_t oud_idx, adc_unit_t unit, adc_channel_t analog_channel);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MODERN_WIND_H__
