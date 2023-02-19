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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <string.h>
#include "press_sen.h"

static const char *TAG = "PRESS_SEN";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

static esp_err_t press_sen_calc_pressure(void *press_sen_sen){
  analog_sen_t *press_sen_sen_ = (analog_sen_t *)press_sen_sen;
  ESP_LOGD(TAG, "press voltage: %f mv", press_sen_sen_->sen.outs[PRESS_SEN_PRESSURE_ID].voltage);
  press_sen_sen_->sen.outs[PRESS_SEN_PRESSURE_ID].pressure = press_sen_sen_->outs[PRESS_SEN_PRESSURE_ID].voltage; //TODO: DO!!!!
  return ESP_OK;
}

static esp_err_t press_sen_calc_temperature(void *press_sen_sen){
  analog_sen_t *press_sen_sen_ = (analog_sen_t *)press_sen_sen;

  return ESP_OK;
}

esp_err_t press_sen_init(analog_sen_t *press_sen_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t press_unit, adc_channel_t press_channel, char *sen_name){
  CHECK_ARG(press_sen_sen);
  CHECK(analog_sen_init_desc(press_sen_sen, samples_filter, period_ms, sen_id, sen_name, 1, press_sen_calc_pressure));
  CHECK(analog_sen_config_output(press_sen_sen, PRESS_SEN_PRESSURE_ID, press_unit, press_channel, NULL));

  press_sen_sen->sen.outs[PRESS_SEN_PRESSURE_ID].out_type = SEN_TYPE_PRESSURE;
  press_sen_sen->sen.outs[PRESS_SEN_PRESSURE_ID].pressure=0.0;
  CHECK(analog_sen_init(press_sen_sen));

  return ESP_OK;
}
