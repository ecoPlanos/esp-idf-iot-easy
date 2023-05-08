/*
 * Copyright (c) 2023 Ecoplanos: Automação e Sistemas Integrados, Lda.
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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include "amphenol_ge_17xx.h"

static const char *TAG = "AMPH_GE_17XX";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

esp_err_t amph_ge_17xx_init(analog_sen_t *amph_ge_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t adc_unit, adc_channel_t adc_channel, char *sen_name){
  CHECK_ARG(amph_ge_sen);
  CHECK(analog_sen_init_desc(amph_ge_sen, samples_filter, period_ms, sen_id, sen_name, 1, amph_ge_17xx_calc_temperature));
  CHECK(analog_sen_config_output(amph_ge_sen, AMPH_GE_17XX_OUT_TEMPERATURE_ID, adc_unit, adc_channel, NULL));

  amph_ge_sen->sen.outs[AMPH_GE_17XX_OUT_TEMPERATURE_ID].out_type = SEN_TYPE_INTERNAL_TEMPERATURE;
  amph_ge_sen->sen.outs[AMPH_GE_17XX_OUT_TEMPERATURE_ID].temperature=0.0;
  CHECK(analog_sen_init(amph_ge_sen));

  return ESP_OK;
}

esp_err_t amph_ge_17xx_calc_temperature(void *amph_ge_sen){
  analog_sen_t *amph_ge_sen_ = (analog_sen_t *)amph_ge_sen;
  // Using f(x) = -0.35825*x^0 + 37.9166*x^1 (good for R2=1K5 and heating systems with start temperature over 25ºC)
  ESP_LOGD(TAG, "temperature voltage: %f mv", amph_ge_sen_->sen.outs[AMPH_GE_17XX_OUT_TEMPERATURE_ID].voltage);
  amph_ge_sen_->sen.outs[AMPH_GE_17XX_OUT_TEMPERATURE_ID].temperature = (amph_ge_sen_->sen.outs[AMPH_GE_17XX_OUT_TEMPERATURE_ID].voltage*0.0379166)-0.35825;
  return ESP_OK;
}
