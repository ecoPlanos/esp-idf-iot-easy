/*
 * Copyright (c) 2022 ecoPlanos <geral@ecoplanos.pt>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file virtual_sensor.c
 *
 * ESP-IDF driver for Sensirion VIRTUAL_SENSOR digital temperature and humidity sensor
 *
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>
#include <string.h>
#include "virtual_sensor.h"

static const char *TAG = "virtual_sensor";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

esp_err_t virtual_sensor_init_desc(virtual_sensor_t *dev, uint16_t sen_id) {
    CHECK_ARG(dev);

    dev->reset_count=0;
    dev->meas_started=false;
    dev->meas_awake_time=0;
    dev->meas_start_time=0;
    dev->meas_get_time=0;

    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,4);
    strncpy(dev->sen.info.name, "VIRTUAL_SEN\0", 12);
    dev->sen.info.lib_id = SEN_VIRTUAL_SENSOR_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.info.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.conf.min_period_us = 1000000;
    dev->sen.conf.delay_start_get_us = 42000;
    dev->sen.info.out_nr = 4;
    dev->sen.conf.trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.conf.period_ms = CONFIG_VIRTUAL_SENSOR_DEFAULT_PERIOD_MS;
    dev->sen.conf.delay_after_awake_us=100000;
    dev->sen.conf.time_to_adjust_us=0;
    dev->sen.dev=dev;
    dev->sen.reset=virtual_sensor_iot_sen_reset;
    dev->sen.reinit=virtual_sensor_iot_sen_reinit;
    dev->sen.start_measurement=virtual_sensor_iot_sen_start_measurement;
    dev->sen.get_data=virtual_sensor_iot_sen_get_data;
    dev->sen.awake=virtual_sensor_iot_sen_sleep_mode_awake;
    dev->sen.sleep=virtual_sensor_iot_sen_sleep_mode_sleep;

    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;

    dev->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].out_id=VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].out_type = SEN_TYPE_TIME;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT32;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].m_raw=0;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].time=0.0;

    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].out_id=VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].out_type = SEN_TYPE_TIME;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT32;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].m_raw=0;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].time=0.0;

    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].out_id=VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].out_type = SEN_TYPE_TIME;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT32;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].m_raw=0;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].time=0.0;

    dev->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].out_id=VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].out_type = SEN_TYPE_COUNTER;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT32;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].m_raw=0;
    dev->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].trig_cnt=0.0;
    dev->sen.conf.srate=0;

    return ESP_OK;
}

esp_err_t virtual_sensor_free_desc(virtual_sensor_t *dev) {
    CHECK_ARG(dev);

    return ESP_OK;
}

esp_err_t virtual_sensor_init(virtual_sensor_t *dev) {
  esp_err_t ret;
  CHECK_ARG(dev);
  dev->sen.status.status_code = SEN_STATUS_OK;
  return ESP_OK;
}

esp_err_t virtual_sensor_sleep(virtual_sensor_t *dev) {
  dev->sleep_mode = VIRTUAL_SENSOR_SLEEP_MODE_SLEEP;
  return ESP_OK;
}

esp_err_t virtual_sensor_awake(virtual_sensor_t *dev) {
  dev->sleep_mode = VIRTUAL_SENSOR_SLEEP_MODE_AWAKE;
  dev->meas_awake_time = esp_timer_get_time();
  return ESP_OK;
}

esp_err_t virtual_sensor_reset(virtual_sensor_t *dev) {
  dev->reset_count++;
  return ESP_OK;
}

esp_err_t virtual_sensor_start_measurement(virtual_sensor_t *dev) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Measurement started!");
    dev->meas_start_time = esp_timer_get_time();
    dev->meas_started = true;
    return ESP_OK;
}

esp_err_t virtual_sensor_iot_sen_start_measurement(void *dev) {
  virtual_sensor_t *dev_ = (virtual_sensor_t *)dev;
  dev_->meas_start_time = esp_timer_get_time();
  dev_->meas_started = true;
  return ESP_OK;
}

esp_err_t virtual_sensor_iot_sen_get_data(void *dev) {
  float temperature, humidity;
  virtual_sensor_t *dev_ = (virtual_sensor_t *)dev;
  vTaskDelay(pdMS_TO_TICKS(100)); //Simulate some delay getting data
  dev_->meas_get_time = esp_timer_get_time();
  dev_->sen.esp_timestamp = esp_timer_get_time();

  dev_->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].m_raw = dev_->meas_awake_time;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].m_raw = dev_->meas_start_time;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].m_raw = dev_->meas_get_time;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].m_raw = dev_->reset_count;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID].time = dev_->meas_awake_time;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID].time = dev_->meas_start_time;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID].time = dev_->meas_get_time;
  dev_->sen.outs[VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID].time = dev_->reset_count;
  return ESP_OK;
}

esp_err_t virtual_sensor_iot_sen_sleep_mode_awake(void *dev) {
  return virtual_sensor_awake((virtual_sensor_t *)dev);
}
esp_err_t virtual_sensor_iot_sen_sleep_mode_sleep(void *dev) {
  return virtual_sensor_sleep((virtual_sensor_t *)dev);
}

esp_err_t virtual_sensor_iot_sen_reset(void *dev) {
  return virtual_sensor_reset((virtual_sensor_t *)dev);
}

esp_err_t virtual_sensor_iot_sen_reinit(void *dev) {
  return virtual_sensor_init((virtual_sensor_t*)dev);
}
