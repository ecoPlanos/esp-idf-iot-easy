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


#ifndef __YF_B6_H__
#define __YF_B6_H__

#include <analog_sen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YF_LIB_VERSION_MAJOR 1
#define YF_LIB_VERSION_MINOR 0
#define YF_LIB_VERSION_PATCH 0
#define YF_LIB_VERSION  (YF_LIB_VERSION_MAJOR << 16)|(YF_LIB_VERSION_MINOR << 8)|YF_LIB_VERSION_PATCH

#define YF_OUT_FLOW_ID 0

/**
 * @brief Initialize device
 *
 * @param yf_sen Analog sensor descriptor
 * @param water_unit ADC unit
 * @param water_channel ADC channel
 * @param sen_name Sensor name
 * @return `ESP_OK` on success
 */
esp_err_t yf_b6_init(analog_sen_t *yf_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t water_unit, adc_channel_t water_channel, char *sen_name);

/**
* @brief Calculate water flow using analog_sen
*
* @param yf_sen Analog sensor descriptor
* @return `ESP_OK` on success
*/
esp_err_t yf_b6_calc_water_flow(void *yf_sen);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __YF_B6_H__
