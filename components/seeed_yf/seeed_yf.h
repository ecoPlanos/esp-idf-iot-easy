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


#ifndef __YF_H__
#define __YF_H__

#include <switch_sen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YF_LIB_VERSION_MAJOR 1
#define YF_LIB_VERSION_MINOR 0
#define YF_LIB_VERSION_PATCH 0
#define YF_LIB_VERSION  (YF_LIB_VERSION_MAJOR << 16)|(YF_LIB_VERSION_MINOR << 8)|YF_LIB_VERSION_PATCH

#define YF_OUT_FLOW_ID 0

typedef enum {
  YF_MODEL_B1 = 0,
  YF_MODEL_B2,
  YF_MODEL_B3,
  YF_MODEL_B4,
  YF_MODEL_B5,
  YF_MODEL_B6,
  YF_MODEL_B7
} yf_model_t;

/**
 * @brief Initialize device
 *
 * @param yf_sen Analog sensor descriptor
 * @param period_ms Period in milliseconds if sensor is read periodicaly
 * @param min_period_us Mininmum period in microseconds ince first transition detection
 * @param sen_id Sensor Id
 * @param water_flow_gpio ESP GPIO number of sensor output
 * @param sen_name Sensor name
 * @param yf_model Sensor model (B1-7)
 * @return `ESP_OK` on success
 */
esp_err_t yf_init(switch_sen_t *yf_sen, uint32_t min_period_us, uint32_t period_ms, uint16_t sen_id, gpio_num_t water_flow_gpio, char *sen_name, yf_model_t yf_model);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __YF_H__
