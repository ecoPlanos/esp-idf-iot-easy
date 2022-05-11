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
#include "yf_b6.h"

#define MIN_FLOW 1.0
#define MAX_FLOW 25.0 //(L/m)
#define FLOW_RESOLUTION_MV (float)CONFIG_YF_B6_SUPPLY_VOLTAGE/(MAX_FLOW - MIN_FLOW + 1.0)

static const char *TAG = "YF-B6";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

esp_err_t yf_b6_init(analog_sen_t *yf_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t water_unit, adc_channel_t water_channel, char *sen_name){
  CHECK_ARG(yf_sen);
  CHECK(analog_sen_init_desc(yf_sen, samples_filter, period_ms, sen_id, sen_name, 1, yf_b6_calc_water_flow));
  CHECK(analog_sen_config_output(yf_sen, YF_OUT_FLOW_ID, water_unit, water_channel, NULL));

  yf_sen->sen.outs[YF_OUT_FLOW_ID].out_type = SEN_TYPE_WATER_FLOW;
  yf_sen->sen.outs[YF_OUT_FLOW_ID].flow=0.0;
  CHECK(analog_sen_init(yf_sen));

  return ESP_OK;
}

esp_err_t yf_b6_calc_water_flow(void *yf_sen){
  analog_sen_t *yf_sen_ = (analog_sen_t *)yf_sen;
  ESP_LOGD(TAG, "water voltage: %f mv", yf_sen_->sen.outs[YF_OUT_FLOW_ID].voltage);
  yf_sen_->sen.outs[YF_OUT_FLOW_ID].flow = (yf_sen_->sen.outs[YF_OUT_FLOW_ID].voltage/0.655963302752)*FLOW_RESOLUTION_MV;
  yf_sen_->sen.outs[YF_OUT_FLOW_ID].m_raw = yf_sen_->sen.outs[YF_OUT_FLOW_ID].voltage/0.655963302752;
  return ESP_OK;
}
