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


#ifndef __SLS_BTA_H__
#define __SLS_BTA_H__

#include <analog_sen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SLS_BTA_LIB_VERSION_MAJOR 1
#define SLS_BTA_LIB_VERSION_MINOR 0
#define SLS_BTA_LIB_VERSION_PATCH 0
#define SLS_BTA_LIB_VERSION  (SLS_BTA_LIB_VERSION_MAJOR << 16)|(SLS_BTA_LIB_VERSION_MINOR << 8)|SLS_BTA_LIB_VERSION_PATCH

enum {
  SLS_BTA_OUT_SOUND_ID = 0,
  SLS_BTA_OUT_REF_ID
};

/**
 * @brief Initialize device
 *
 * @param sls_bta_sen Analog sensor descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sls_bta_init(analog_sen_t *sls_bta_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t sound_unit, adc_channel_t sound_channel, adc_unit_t ref_unit, adc_channel_t ref_channel, char *sen_name);

/**
* @brief Free device descriptor
*
* @param sls_bta_sen Analog sensor descriptor
* @param oud_idx Analog output index
* @param unit ADC unit
* @param analog_channel ADC channel
* @return `ESP_OK` on success
*/
esp_err_t sls_bta_config_output(analog_sen_t *sls_bta_sen, uint8_t oud_idx, adc_unit_t unit, adc_channel_t analog_channel);

esp_err_t sls_bta_calc_sound_pressure(void *sls_bta_sen);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SLS_BTA_H__
