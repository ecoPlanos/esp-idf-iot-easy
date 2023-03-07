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

static inline esp_err_t modern_wind_calc_temperature(void *modern_wind_sen){
  modern_wind_t *modern_wind_sen_ = (modern_wind_t *)modern_wind_sen;
  modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_TEMP_ID].temperature = ((float)modern_wind_sen_->analog_sen.outs[MODERN_WIND_TEMP_ID].voltage - modern_wind_sen_->conf.v0) / modern_wind_sen_->conf.tc; // ℃
  // modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].temperature = (modern_wind_sen_->outs[MODERN_WIND_TEMP_ID].voltage - MODERN_WIND_TEMPERATURE_V0) / MODERN_WIND_TEMPERATURE_TC; // m/s
  ESP_LOGD(TAG, "Temperature: %f", modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_TEMP_ID].temperature);

  return ESP_OK;
}

static inline esp_err_t modern_wind_calc_wind_speed(void *modern_wind_sen){
  modern_wind_t *modern_wind_sen_ = (modern_wind_t *)modern_wind_sen;
  ESP_LOGD(TAG, "wind voltage: %u mv", modern_wind_sen_->analog_sen.outs[MODERN_WIND_WIND_ID].voltage);
  ESP_LOGD(TAG, "temperature voltage: %u mv", modern_wind_sen_->analog_sen.outs[MODERN_WIND_TEMP_ID].voltage);
  modern_wind_calc_temperature(modern_wind_sen);
  if(modern_wind_sen_->analog_sen.outs[MODERN_WIND_WIND_ID].voltage < modern_wind_sen_->conf.zero_wind_mv) {
    ESP_LOGW(TAG, "Wind speed invalid!");
    ESP_LOGD(TAG, "Zero wind voltage: %f mV", modern_wind_sen_->conf.zero_wind_mv);
    ESP_LOGD(TAG, "Output voltage: %u mV", modern_wind_sen_->analog_sen.outs[MODERN_WIND_WIND_ID].voltage);
    modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_WIND_ID].wind_speed = 0.0;
  } else {
    ESP_LOGD(TAG, "wind_speed: %f", pow(((((float)modern_wind_sen_->analog_sen.outs[MODERN_WIND_WIND_ID].voltage - modern_wind_sen_->conf.zero_wind_mv)/(3038.517 * pow(modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_TEMP_ID].temperature, 0.115157))) / 0.087288), 3.009364) * 0.44704);
    modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_WIND_ID].wind_speed = pow(\
      ((((float)modern_wind_sen_->analog_sen.outs[MODERN_WIND_WIND_ID].voltage - modern_wind_sen_->conf.zero_wind_mv)\
      / (3038.517 * pow(modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_TEMP_ID].temperature, 0.115157))) / 0.087288), 3.009364) * 0.44704; // m/s
    // modern_wind_sen_->sen.outs[MODERN_WIND_WIND_ID].wind_speed = pow((((modern_wind_sen_->outs[MODERN_WIND_WIND_ID].voltage - MODERN_WIND_ZERO_WIND_MV) / (3038.517 * pow(modern_wind_sen_->sen.outs[MODERN_WIND_TEMP_ID].temperature, 0.115157))) / 0.087288), 3.009364) * 0.44704; // m/s
    // modern_wind_sen_->analog_sen.sen.outs[MODERN_WIND_WIND_ID].wind_speed = (modern_wind_sen_->analog_sen.outs[MODERN_WIND_WIND_ID].voltage - modern_wind_sen_->conf.zero_wind_mv);
  }
  return ESP_OK;
}

esp_err_t modern_wind_init(modern_wind_t *modern_wind_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t wind_unit, adc_channel_t wind_channel, adc_unit_t tmp_unit, adc_channel_t tmp_channel, gpio_num_t shdn_gpio, char *sen_name){
  ESP_LOGI(TAG, "MODERN_WIND initializing...");
  CHECK_ARG(modern_wind_sen);
  memset(&modern_wind_sen->analog_sen, 0, sizeof(analog_sen_t));
  // CHECK(analog_sen_init_desc(modern_wind_sen, samples_filter, period_ms, sen_id, sen_name, 2, NULL));
  CHECK(analog_sen_init_desc(&modern_wind_sen->analog_sen, samples_filter, period_ms, sen_id, sen_name, 2, modern_wind_calc_wind_speed));
  // CHECK(analog_sen_config_output(modern_wind_sen, MODERN_WIND_WIND_ID, wind_unit, wind_channel, modern_wind_calc_wind_speed));
  CHECK(analog_sen_config_output(&modern_wind_sen->analog_sen, MODERN_WIND_WIND_ID, wind_unit, wind_channel, NULL));
  // CHECK(analog_sen_config_output(modern_wind_sen, MODERN_WIND_TEMP_ID, tmp_unit, tmp_channel, modern_wind_calc_temperature));
  CHECK(analog_sen_config_output(&modern_wind_sen->analog_sen, MODERN_WIND_TEMP_ID, tmp_unit, tmp_channel, NULL));

  // modern_wind_sen->conf.zero_wind_mv=1369.2;
  modern_wind_sen->conf.zero_wind_mv=1254.33333333333;
  modern_wind_sen->conf.tc=19.5;
  modern_wind_sen->conf.v0=400.0;
  modern_wind_sen->conf.shdn_gpio=shdn_gpio;
  modern_wind_sen->analog_sen.sen.awake=modern_wind_iot_sen_sleep_mode_awake;
  modern_wind_sen->analog_sen.sen.sleep=modern_wind_iot_sen_sleep_mode_sleep;
  modern_wind_sen->analog_sen.sen.conf.min_period_us = 5000000;
  modern_wind_sen->analog_sen.sen.conf.delay_start_get_us = 1000000;
  modern_wind_sen->analog_sen.sen.conf.delay_after_awake_us = 100000;
#ifdef CONFIG_MODERN_WIND_USE_HW_CTRL
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = (1ULL<<modern_wind_sen->conf.shdn_gpio);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  CHECK(gpio_config(&io_conf));
  CHECK(gpio_set_level(modern_wind_sen->conf.shdn_gpio 0));
#endif
  modern_wind_sen->analog_sen.sen.outs[MODERN_WIND_WIND_ID].out_type = SEN_TYPE_WIND;
  modern_wind_sen->analog_sen.sen.outs[MODERN_WIND_WIND_ID].wind_speed=0.0;
  modern_wind_sen->analog_sen.sen.outs[MODERN_WIND_TEMP_ID].out_type = SEN_TYPE_AMBIENT_TEMPERATURE;
  modern_wind_sen->analog_sen.sen.outs[MODERN_WIND_TEMP_ID].temperature=0.0;
  CHECK(analog_sen_init(&modern_wind_sen->analog_sen));

  return ESP_OK;
}

esp_err_t modern_wind_set_sleep_mode(modern_wind_t *dev, modern_wind_sleep_type_t sleep_mode) {
  uint8_t cmd[7];
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "modern_wind_set_sleep_mode");
  if(dev->status.sleep_mode == sleep_mode) return ESP_OK;
  if((sleep_mode!=MODERN_WIND_SLEEP_MODE_SLEEP) && (sleep_mode!=MODERN_WIND_SLEEP_MODE_AWAKE)){
    ESP_LOGE(TAG, "Ivalid sleep mode!");
    return ESP_ERR_INVALID_STATE;
  }
#ifdef CONFIG_MODERN_WIND_USE_HW_CTRL
  if(sleep_mode==MODERN_WIND_SLEEP_MODE_SLEEP) {
    ESP_LOGD(TAG, "Sensor going to sleep");
    CHECK(gpio_set_level(dev->conf.shdn_gpio, 0));
  } else {
    ESP_LOGD(TAG, "Sensor awaking");
    CHECK(gpio_set_level(dev->conf.shdn_gpio, 1));
  }
#endif
  dev->status.sleep_mode = sleep_mode;
  ESP_LOGD(TAG, "Current sleep mode: %u", dev->status.sleep_mode);
  return ESP_OK;
}

esp_err_t modern_wind_iot_sen_sleep_mode_awake(void *dev) {
  return modern_wind_set_sleep_mode((modern_wind_t *)dev, MODERN_WIND_SLEEP_MODE_AWAKE);
}

esp_err_t modern_wind_iot_sen_sleep_mode_sleep(void *dev) {
  return modern_wind_set_sleep_mode((modern_wind_t *)dev, MODERN_WIND_SLEEP_MODE_SLEEP);
}
