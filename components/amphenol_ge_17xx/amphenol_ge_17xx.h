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


#ifndef __AMPH_GE_17XX_H__
#define __AMPH_GE_17XX_H__

#include <analog_sen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AMPH_GE_17XX_LIB_VERSION_MAJOR 1
#define AMPH_GE_17XX_LIB_VERSION_MINOR 0
#define AMPH_GE_17XX_LIB_VERSION_PATCH 0
#define AMPH_GE_17XX_LIB_VERSION  (AMPH_GE_17XX_LIB_VERSION_MAJOR << 16)|(AMPH_GE_17XX_LIB_VERSION_MINOR << 8)|AMPH_GE_17XX_LIB_VERSION_PATCH

enum {
  AMPH_GE_17XX_OUT_TEMPERATURE_ID = 0,
};

/**
 * @brief Initialize device
 *
 * @param amph_ge_17xx_sen Analog sensor descriptor
 * @return `ESP_OK` on success
 */
esp_err_t amph_ge_17xx_init(analog_sen_t *amph_ge_17xx_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t adc_unit, adc_channel_t adc_channel, char *sen_name);

esp_err_t amph_ge_17xx_config_output(analog_sen_t *amph_ge_17xx_sen, uint8_t oud_idx, adc_unit_t unit, adc_channel_t analog_channel);

esp_err_t amph_ge_17xx_calc_temperature(void *amph_ge_17xx_sen);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __AMPH_GE_17XX_H__
