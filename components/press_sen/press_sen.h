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


#ifndef __PRESS_SEN_H__
#define __PRESS_SEN_H__

#include <analog_sen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PRESS_SEN_LIB_VERSION_MAJOR 1
#define PRESS_SEN_LIB_VERSION_MINOR 0
#define PRESS_SEN_LIB_VERSION_PATCH 0
#define PRESS_SEN_LIB_VERSION  (PRESS_SEN_LIB_VERSION_MAJOR << 16)|(PRESS_SEN_LIB_VERSION_MINOR << 8)|PRESS_SEN_LIB_VERSION_PATCH

enum {
  PRESS_SEN_PRESSURE_ID = 0
};

typedef struct {
  uint16_t zero_press_mv;
}press_sen_conf_t;
/**
 * @brief Initialize device
 *
 * @param press_sen_sen Analog sensor descriptor
 * @return `ESP_OK` on success
 */
esp_err_t press_sen_init(analog_sen_t *press_sen_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t press_unit, adc_channel_t press_channel, char *sen_name);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __PRESS_SEN_H__
