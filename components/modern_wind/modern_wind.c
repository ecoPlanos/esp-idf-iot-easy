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
#include <math.h>
#include <string.h>
#include "modern_wind.h"

static const char *TAG = "MODERN_WIND";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

static esp_err_t modern_wind_calc_temperature(void *modern_wind_sen){
  analog_sen_t *modern_wind_sen_ = (analog_sen_t *)modern_wind_sen;
  // modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].temperature = (modern_wind_sen_->outs[MODERN_WIND_WIND_ID].voltage - modern_wind_sen_->config.v0) / modern_wind_sen_->config.tc; // m/s
  modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].temperature = (modern_wind_sen_->outs[MODERN_WIND_WIND_ID].voltage - MODERN_WIND_TEMPERATURE_V0) / MODERN_WIND_TEMPERATURE_TC; // m/s

  return ESP_OK;
}

static esp_err_t modern_wind_calc_wind_speed(void *modern_wind_sen){
  analog_sen_t *modern_wind_sen_ = (analog_sen_t *)modern_wind_sen;
  ESP_LOGD(TAG, "wind voltage: %f mv", modern_wind_sen_->sen.outs[MODERN_WIND_WIND_ID].voltage);
  ESP_LOGD(TAG, "temperature voltage: %f mv", modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].voltage);
  modern_wind_calc_temperature(modern_wind_sen);
  // modern_wind_sen_->sen.outs[MODERN_WIND_WIND_ID].wind_speed = pow((((modern_wind_sen_->outs[MODERN_WIND_WIND_ID].voltage - modern_wind_sen_->config.zero_wind_mv) / (3038.517 * pow(modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].temperature, 0.115157))) / 0.087288), 3.009364) * 0.44704; // m/s
  modern_wind_sen_->sen.outs[MODERN_WIND_WIND_ID].wind_speed = pow((((modern_wind_sen_->outs[MODERN_WIND_WIND_ID].voltage - MODERN_WIND_ZERO_WIND_MV) / (3038.517 * pow(modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].temperature, 0.115157))) / 0.087288), 3.009364) * 0.44704; // m/s

  return ESP_OK;
}

esp_err_t modern_wind_init(analog_sen_t *modern_wind_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t wind_unit, adc_channel_t wind_channel, adc_unit_t tmp_unit, adc_channel_t tmp_channel, gpio_num_t shdn_gpio, char *sen_name){
  CHECK_ARG(modern_wind_sen);
  CHECK(analog_sen_init_desc(modern_wind_sen, samples_filter, period_ms, sen_id, sen_name, 2, NULL));
  CHECK(analog_sen_config_output(modern_wind_sen, MODERN_WIND_WIND_ID, wind_unit, wind_channel, modern_wind_calc_wind_speed));
  // CHECK(analog_sen_config_output(modern_wind_sen, MODERN_WIND_TEMP_ID, tmp_unit, tmp_channel, modern_wind_calc_temperature));
  CHECK(analog_sen_config_output(modern_wind_sen, MODERN_WIND_TEMP_ID, tmp_unit, tmp_channel, NULL));
  // modern_wind_sen->config.zero_wind_mv = 1369.2;
  // modern_wind_sen->config.tc = 19.5;
  // modern_wind_sen->config.v0 = 400;
#ifdef CONFIG_MODERN_WIND_USE_HW_CTRL
  gpio_config_t io_conf;
  ESP_LOGI(TAG, "PMS1003 initializing...");
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = (1ULL<<shdn_gpio);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
#endif
  modern_wind_sen->sen.outs[MODERN_WIND_WIND_ID].out_type = SEN_TYPE_WIND;
  modern_wind_sen->sen.outs[MODERN_WIND_WIND_ID].wind_speed=0.0;
  modern_wind_sen->sen.outs[MODERN_WIND_TEMP_ID].out_type = SEN_TYPE_AMBIENT_TEMPERATURE;
  modern_wind_sen->sen.outs[MODERN_WIND_TEMP_ID].temperature=0.0;
  CHECK(analog_sen_init(modern_wind_sen));

  return ESP_OK;
}
