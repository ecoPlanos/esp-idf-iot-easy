/*
 * Copyright (c) 2021 Ecoplanos: Automação e Sistemas Integrados, Lda.
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

#include <data_manager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <stdlib.h>
#include <string.h>
#include <esp_timer.h>
#include "switch_sen.h"

static const char *TAG = "switch_sen";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  // sensor_t *sen = (sensor_t*) arg;
  uint8_t gpio_num = (uint8_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void sw_trigger_task(void* arg) {
  sensor_t *sen = (sensor_t *)arg;
  bool detect_running = false;
  int64_t current_timestamp, current_trig = 0;
  uint32_t current_filter_cnt = 0;
  uint8_t io_num;
  for(;;) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      if(io_num==sen->outs[0].gpio){
        current_timestamp = esp_timer_get_time();
        if(detect_running){
          if((current_timestamp - current_trig) > sen->conf.min_period_us) {
            detect_running = false;
            sen->outs[0].m_raw = (uint32_t)(current_timestamp - current_trig);
            sen->outs[0].trig_cnt = current_filter_cnt;
            sen->esp_timestamp = current_trig;
            ESP_LOGD(TAG, "end of detection with cntr: %u and duration: %u us",sen->outs[0].trig_cnt,sen->outs[0].m_raw);
          } else {
            current_filter_cnt++;
          }
        } else {
          ESP_LOGD(TAG, "started new detection");
          detect_running = true;
          current_trig = current_timestamp;
          current_filter_cnt = 1;
        }
      }
    }
  }
}

static esp_err_t switch_iot_sen_reset(void *dev) {
  return ESP_OK;
}
static esp_err_t switch_iot_sen_reinit(void *dev) {
  return ESP_OK;
}
static esp_err_t switch_iot_sen_start_measurement(void *dev) {
  return ESP_OK;
}
static esp_err_t switch_iot_sen_sleep_mode_awake(void *dev) {
  return ESP_OK;
}
static esp_err_t switch_iot_sen_sleep_mode_sleep(void *dev) {
  return ESP_OK;
}

static esp_err_t switch_iot_sen_get_data(void *dev) {
  switch_sen_t *switch_dev = (switch_sen_t *)dev;
  uint8_t i;
  ESP_LOGD(TAG, "switch_iot_sen_get_data");
  // for(i=0;i<switch_dev->outs_nr;i++){
  //   if(switch_dev->outs[i].calc_processed != NULL)
  //     switch_dev->outs[i].calc_processed(switch_dev);
  // }
  if(switch_dev->calc_processed != NULL)
    switch_dev->calc_processed(switch_dev);

  return ESP_OK;
}

esp_err_t switch_sen_init(switch_sen_t *dev, sen_out_trig_dir_type_t trigger_dir, uint32_t min_period_us, gpio_num_t input_pin, gpio_pullup_t pull_up_en,gpio_pulldown_t pull_down_en, uint8_t dev_id, uint8_t pack_id, uint8_t sen_id, char sen_name[], void *calc_processed) {
  CHECK_ARG(dev);
  esp_err_t ret;
  ESP_LOGI(TAG,"Initializing switch sensor descriptor");
  dev->calc_processed = calc_processed;
  dev->conf.gpio = input_pin;
  dev->conf.trig_dir = trigger_dir;
  dev->conf.min_period_us = min_period_us;
  dev->conf.ver = 0;

  memset(&dev->sen, 0, sizeof(sensor_t));
  sensor_init(&dev->sen,1);
  strcpy(dev->sen.info.name, sen_name);
  dev->sen.info.lib_id = SEN_SW_LIB_ID;
  dev->sen.info.sen_id = sen_id;
  dev->sen.info.sen_lib_version = SWITCH_SEN_LIB_VERSION;
  dev->sen.info.version = 1;
  dev->sen.info.com_type = SEN_COM_TYPE_DIGITAL;
  dev->sen.conf.min_period_us = min_period_us;
  dev->sen.conf.delay_start_get_us = 0;
  dev->sen.info.out_nr = 1;
  dev->sen.info.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_EVENT;
  dev->sen.conf.addr = 0;
  dev->sen.conf.period_ms=0;
  dev->sen.reset=switch_iot_sen_reset;
  dev->sen.reinit=switch_iot_sen_reinit;
  dev->sen.start_measurement=switch_iot_sen_start_measurement;
  dev->sen.get_data=switch_iot_sen_get_data;
  dev->sen.awake=switch_iot_sen_sleep_mode_awake;
  dev->sen.sleep=switch_iot_sen_sleep_mode_sleep;
  dev->sen.dev=dev;

  dev->sen.status.fail_cnt = 0;
  dev->sen.status.fail_time = 0;

  dev->sen.outs[0].out_id=0;
  dev->sen.outs[0].gpio = input_pin;
  dev->sen.outs[0].out_type = SEN_TYPE_SWITCH;
  dev->sen.outs[0].out_trigger_dir = trigger_dir;
  dev->sen.outs[0].out_val_type=SEN_OUT_VAL_TYPE_SEN_SWITCH;
  dev->sen.outs[0].m_raw=0;
  dev->sen.conf.srate=0;

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1ULL<<input_pin);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = pull_up_en;
  io_conf.pull_down_en = pull_down_en;
  CHECK(gpio_config(&io_conf));
  gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));
  if(!gpio_evt_queue) return ESP_FAIL;
  ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
  ret = (ret==ESP_ERR_INVALID_STATE) ? ESP_OK : ret;  //ISR service already running...
  CHECK(ret);
  ret = gpio_isr_handler_add(input_pin, gpio_isr_handler, (void *) input_pin);
  BaseType_t task_return = xTaskCreate(&sw_trigger_task, dev->sen.info.name, 2048, (void *) (&dev->sen), 3|portPRIVILEGE_BIT, &dev->sen.outs[0].task_handle);
  configASSERT(dev->sen.outs[0].task_handle);
  if( task_return == pdPASS ) {
    ESP_LOGI(TAG, "Sensor %s event trigger task is running.",dev->sen.info.name);
    ret = ESP_OK;
  }
  if(ret==ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_OK;
  }else{
    dev->sen.status.status_code=SEN_STATUS_FAIL_INIT;
  }
  return ret;
}

esp_err_t switch_sen_free(switch_sen_t *dev) {
  // TODO: remove interrupts delete tasks
  return ESP_OK;
}

esp_err_t switch_sen_get_val(switch_sen_t *dev) {
  return gpio_get_level(dev->conf.gpio);
}
// esp_err_t switch_sen_iot_sen_measurement(void *dev) {
//   //TODO: get latest sensor data
//   return ESP_OK;
// }
