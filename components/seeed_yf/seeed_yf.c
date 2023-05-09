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
#include "seeed_yf.h"

static const float YF_FACTOR[7] = {11.0, 11.0, 11.0, 11.0, 6.6, 6.6, 11.0};

static const char *TAG = "seeed_YF";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

static void yf_calc_water_flow(void *yf_sen){
  switch_sen_t *yf_sen_ = (switch_sen_t *)yf_sen;
  uint32_t dur, cnt;
  dur = yf_sen_->trig_duration;
  cnt = yf_sen_->trig_cnt;
  yf_sen_->trig_duration-=dur; 
  yf_sen_->trig_cnt-=cnt;
  ESP_LOGD(TAG, "trigger_cnt: %u times", cnt);
  ESP_LOGD(TAG, "tirg_duration: %u us", dur);
  ESP_LOGD(TAG, "using factor: %f", YF_FACTOR[(yf_model_t)yf_sen_->info.model]);
  if((cnt == 0) || (dur == 0)) {
    yf_sen_->sen.outs[YF_OUT_FLOW_ID].flow = 0.0;
    return;
  }
  yf_sen_->sen.outs[YF_OUT_FLOW_ID].flow = ((((float)cnt)/2.0)/\
                                            (((float)dur)/1000000.0))/\
                                            YF_FACTOR[(yf_model_t)yf_sen_->info.model];
  ESP_LOGD(TAG, "L/m: %f", yf_sen_->sen.outs[YF_OUT_FLOW_ID].flow);
}

esp_err_t yf_init(switch_sen_t *yf_sen, uint32_t min_period_us, uint32_t period_ms, uint16_t sen_id, gpio_num_t water_flow_gpio, char *sen_name, yf_model_t yf_model){
  CHECK_ARG(yf_sen);
  CHECK(switch_sen_init(yf_sen, SEN_OUT_TRIGGER_RE, min_period_us, 0, water_flow_gpio, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_ENABLE, 0, 0, sen_id, sen_name, SWITCH_TYPE_COUNTER, yf_calc_water_flow));
  // yf_sen->sen.conf.period_ms = period_ms;
  // yf_sen->sen.info.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
  yf_sen->sen.outs[YF_OUT_FLOW_ID].out_type = SEN_TYPE_WATER_FLOW;
  yf_sen->sen.outs[YF_OUT_FLOW_ID].flow=0.0;
  yf_sen->info.model=yf_model;

  return ESP_OK;
}
