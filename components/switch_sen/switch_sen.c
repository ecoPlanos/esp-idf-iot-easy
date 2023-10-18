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

#include <cJSON.h>
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
  // sensor_t *sen = (sensor_t *)arg;
  switch_sen_t *dev = (switch_sen_t *)arg;
  bool detect_running = false;
  int64_t current_timestamp, trig_on = 0, trig_off = 0, detect_start=0;
  uint8_t io_num;
  for(;;) {
    if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
      if(io_num==dev->sen.outs[0].gpio){
        current_timestamp = esp_timer_get_time();
        dev->trig_cnt ++;
        if(detect_running){
          trig_off = current_timestamp;
          dev->on_dur=(uint32_t)(current_timestamp - trig_on);
          if(dev->conf.sw_type == SWITCH_TYPE_STATE) {
            dev->state = SEN_OUT_STATE_OFF;
          } else {  // Other switch types
            if(dev->on_dur < dev->sen.conf.min_period_us) {
              dev->trig_duration += dev->on_dur;
              if(dev->trig_cnt>2) {
                dev->frequency = (1000000.0/((float)((float)dev->on_dur+(float)dev->off_dur)));
                ESP_LOGD(TAG, "total duration: %u us, total trigers: %u, frequency: %f",dev->trig_duration,dev->trig_cnt,dev->frequency);
              }
            } else {
              detect_start = current_timestamp;
              ESP_LOGE(TAG, "Detected end of measurement!");
              dev->trig_duration = 0;
              dev->frequency = 0.0;
              dev->trig_cnt = 1;
            }
          }
          dev->sen.esp_timestamp = current_timestamp;
          detect_running = false;
          ESP_LOGD(TAG,"on_dur: %u", dev->on_dur);
        } else {  // !detect_running
          dev->off_dur=(uint32_t)(current_timestamp - trig_off);
          if(dev->conf.sw_type == SWITCH_TYPE_STATE) {
            if(((dev->sen.outs[0].out_trigger_dir==SEN_OUT_TRIGGER_RE) && gpio_get_level(dev->conf.gpio)) || \
               ((dev->sen.outs[0].out_trigger_dir==SEN_OUT_TRIGGER_FE) && (!gpio_get_level(dev->conf.gpio)))) {
              trig_on = current_timestamp;
              dev->state = SEN_OUT_STATE_ON;
              dev->sen.esp_timestamp = current_timestamp;
              detect_running = true;
            } else {
              ESP_LOGD(TAG, "detected oposite transition");
              if(dev->state != SEN_OUT_STATE_OFF) {
                trig_off = current_timestamp;
                dev->state = SEN_OUT_STATE_OFF;
                dev->sen.esp_timestamp = current_timestamp;
              }
            }
          } else {  // Other switch types
            trig_on = current_timestamp;
            if(dev->off_dur < dev->sen.conf.min_period_us) {
              dev->trig_duration += dev->off_dur;
              if(dev->trig_cnt>2) {
                dev->frequency = (1000000.0/((float)((float)dev->on_dur+(float)dev->off_dur)));
                dev->sen.esp_timestamp = current_timestamp;
                ESP_LOGD(TAG, "total duration: %u us, total trigers: %u, frequency: %f",dev->trig_duration,dev->trig_cnt,dev->frequency);
              }
            } else {
              detect_start = current_timestamp;
              ESP_LOGE(TAG, "Detected end of measurement!");
              dev->trig_duration = 0;
              dev->frequency = 0.0;
              dev->trig_cnt = 1;
            }
            detect_running = true;
            dev->sen.esp_timestamp = current_timestamp;
            ESP_LOGD(TAG,"off_dur: %u", dev->off_dur);
          }
        }
      }
    }
  }
}

static esp_err_t switch_iot_sen_reset(void *dev) {
  ESP_LOGD(TAG,"IOT reset");
  return ESP_OK;
}
static esp_err_t switch_iot_sen_reinit(void *dev) {
  ESP_LOGD(TAG,"IOT reinit");
  return ESP_OK;
}
static esp_err_t switch_iot_sen_start_measurement(void *dev_) {
  switch_sen_t *dev = (switch_sen_t *)dev_;
  ESP_LOGD(TAG,"IOT start measurement");
  switch(dev->conf.sw_type) {
    case SWITCH_TYPE_ACTUATOR:
    case SWITCH_TYPE_STATE:
      dev->state = ((dev->sen.outs[0].out_trigger_dir==SEN_OUT_TRIGGER_RE)?gpio_get_level(dev->conf.gpio):!gpio_get_level(dev->conf.gpio));
      dev->esp_timestamp = esp_timer_get_time();
      ESP_LOGD(TAG, "Measured state: %u",dev->state);
    break;
    case SWITCH_TYPE_ACTUATOR_PWM:
      return ESP_OK;
    break;
    case SWITCH_TYPE_COUNTER:
      return ESP_OK;
    break;
    default:
      ESP_LOGE(TAG,"Invalid switch type!");
      return ESP_ERR_INVALID_ARG;
    break;
  }
  return ESP_OK;
}
static esp_err_t switch_iot_sen_sleep_mode_awake(void *dev) {
  ESP_LOGD(TAG,"IOT awake");
  return ESP_OK;
}
static esp_err_t switch_iot_sen_sleep_mode_sleep(void *dev) {
  ESP_LOGD(TAG,"IOT sleep");
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
  
  switch(switch_dev->conf.sw_type) {
    case SWITCH_TYPE_ACTUATOR:
    case SWITCH_TYPE_STATE:
      switch_dev->sen.outs[0].processed = switch_dev->state;
    break;
    case SWITCH_TYPE_ACTUATOR_PWM:
      switch_dev->sen.outs[0].processed = switch_dev->pwm;
    break;
    case SWITCH_TYPE_COUNTER:
      if(switch_dev->calc_processed != NULL) {
        ESP_LOGD(TAG, "post processing data...");
        switch_dev->calc_processed(switch_dev);
        return ESP_OK;
      } else {
        switch_dev->sen.outs[0].processed = switch_dev->trig_cnt;
      // switch_dev->sen.outs[1].duration = switch_dev->trig_duration;
      }
    break;
    default:
      ESP_LOGE(TAG,"Invalid switch type!");
      return ESP_ERR_INVALID_ARG;
    break;
  }
  if(switch_dev->calc_processed != NULL) {
    ESP_LOGD(TAG, "post processing data...");
    switch_dev->calc_processed(switch_dev);
  } 
  return ESP_OK;
}

static esp_err_t switch_iot_sen_parce_command(void* dev_, char *cmd, size_t cmd_len) {
  switch_sen_t *dev = (switch_sen_t *)dev_;
  char *json_str = NULL;
	size_t json_str_len;
  esp_err_t ret = ESP_OK;
  if(cmd) {
    cJSON *cmd_json, *command_args;
    size_t args_nr;
    char* command;
    ESP_LOGD(TAG, "Processing MQTT Data");
    cmd_json = cJSON_ParseWithLength(cmd,cmd_len);
    if(cmd_json) {
      command = cJSON_GetStringValue(cJSON_GetObjectItem(cmd_json,"command"));
      if ( strcmp(command, SWITCH_COMMAND_SET_STATE) == 0 ) {
        ESP_LOGI(TAG, "Setting switch state");
        command_args=cJSON_GetObjectItem(cmd_json, "args");
        if(!command_args) {
          ESP_LOGD(TAG, "command without arguments");
          cJSON_Delete(cmd_json);
          return ESP_ERR_INVALID_ARG;
        }
        ESP_LOGD(TAG, "Received command arguments");
        args_nr=cJSON_GetArraySize(command_args);
        if(args_nr<1) {
          ESP_LOGE(TAG, "instruction accepts 1 argument (on/off)");
          cJSON_Delete(cmd_json);
          return ESP_ERR_INVALID_ARG;
        }
        sen_out_state_t state = (sen_out_state_t) cJSON_GetNumberValue(cJSON_GetArrayItem(command_args, 0));
        ESP_LOGD(TAG, "State: %u",state);
        if(!((state == SEN_OUT_STATE_ON) || (state == SEN_OUT_STATE_OFF))) {
          ESP_LOGE(TAG, "Invalid state (%u). It must be 0 or 1", state);
          cJSON_Delete(cmd_json);
          return ESP_FAIL;
        }
        return switch_sen_set_state(dev, state);
      } else if (strcmp(command, SWITCH_COMMAND_SET_PWM) == 0) {
            ESP_LOGI(TAG, "Setting switch pwm");
      } else {
        ESP_LOGE(TAG, "Invalid command for switch component");
        cJSON_Delete(cmd_json);
        return ESP_ERR_INVALID_ARG;
      }  
    } else {
      ESP_LOGE(TAG, "Invalid command JSON.");
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    ESP_LOGE(TAG, "cmd function argument is not initialized");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

esp_err_t switch_sen_init(switch_sen_t *dev, sen_out_trig_dir_type_t trigger_dir, uint32_t min_period_us, uint32_t period_ms, gpio_num_t io_pin, gpio_pullup_t pull_up_en,gpio_pulldown_t pull_down_en, uint8_t dev_id, uint8_t pack_id, uint8_t sen_id, char sen_name[], switch_type_t switch_type, void *calc_processed) {
  CHECK_ARG(dev);
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG,"Initializing switch sensor descriptor");
  dev->pwm = 0;
  dev->state = 0;
  dev->trig_duration = 0;
  dev->trig_cnt = 0;
  dev->on_dur = UINT32_MAX;
  dev->off_dur = UINT32_MAX;
  dev->frequency = 0.0;
  dev->esp_timestamp = 0;
  dev->calc_processed = calc_processed;
  dev->conf.gpio = io_pin;
  dev->conf.trig_dir = trigger_dir;
  dev->conf.min_period_us = min_period_us;
  dev->conf.ver = 0;
  dev->conf.sw_type = switch_type;

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
  dev->sen.conf.trigger_type = ((period_ms > 0) ? SEN_OUT_TRIGGER_TYPE_TIME : SEN_OUT_TRIGGER_TYPE_EVENT);
  dev->sen.conf.addr = 0;
  dev->sen.conf.period_ms=period_ms;
  dev->sen.reset=switch_iot_sen_reset;
  dev->sen.reinit=switch_iot_sen_reinit;
  dev->sen.start_measurement=switch_iot_sen_start_measurement;
  dev->sen.get_data=switch_iot_sen_get_data;
  dev->sen.awake=switch_iot_sen_sleep_mode_awake;
  dev->sen.sleep=switch_iot_sen_sleep_mode_sleep;
  dev->sen.parce_cmd=switch_iot_sen_parce_command;
  dev->sen.dev=dev;

  dev->sen.status.fail_cnt = 0;
  dev->sen.status.fail_time = 0;
  // TODO: Add outputs for all possilbe switch outputs depending on switch type, instead of using mraw to store extra information!!!
  dev->sen.outs[0].out_id=0;
  dev->sen.outs[0].gpio = io_pin;
  dev->sen.outs[0].out_trigger_dir = trigger_dir;
  dev->sen.outs[0].out_val_type=SEN_OUT_VAL_TYPE_BOOL;
  dev->sen.outs[0].m_raw=0;
  dev->sen.conf.srate=0;

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1ULL<<io_pin);
  if(switch_type==SWITCH_TYPE_ACTUATOR_PWM) {
    io_conf.mode = GPIO_MODE_OUTPUT;
  } else if(switch_type==SWITCH_TYPE_ACTUATOR) {
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
  } else {
    io_conf.mode = GPIO_MODE_INPUT;
  }
  io_conf.pull_up_en = pull_up_en;
  io_conf.pull_down_en = pull_down_en;
  CHECK(gpio_config(&io_conf));

  if(switch_type==SWITCH_TYPE_STATE || switch_type==SWITCH_TYPE_COUNTER) {
    gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));
    if(!gpio_evt_queue) return ESP_FAIL;
    ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    ret = (ret==ESP_ERR_INVALID_STATE) ? ESP_OK : ret;  //ISR service already running...
    CHECK(ret);
    ret = gpio_isr_handler_add(io_pin, gpio_isr_handler, (void *) io_pin);
    BaseType_t task_return = xTaskCreate(&sw_trigger_task, dev->sen.info.name, 2048, (void *) (dev), 3|portPRIVILEGE_BIT, &dev->sen.outs[0].task_handle);
    configASSERT(dev->sen.outs[0].task_handle);
    if( task_return == pdPASS ) {
      ESP_LOGI(TAG, "Sensor %s event trigger task is running.",dev->sen.info.name);
      ret = ESP_OK;
    }
  }
  if(ret==ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_OK;
    // dev->sen.status.initialized=true;
  }else{
    dev->sen.status.status_code=SEN_STATUS_FAIL_INIT;
  }
  return ret;
}

esp_err_t switch_sen_free(switch_sen_t *dev) {
  // TODO: remove interrupts delete tasks
  return ESP_OK;
}

esp_err_t switch_sen_set_state(switch_sen_t *dev, sen_out_state_t state) {
  if(dev->conf.sw_type != SWITCH_TYPE_ACTUATOR) {
    ESP_LOGE(TAG,"switch device must be actuator type");
    return ESP_ERR_INVALID_ARG;
  }
  bool level=(dev->sen.outs[0].out_trigger_dir==SEN_OUT_TRIGGER_RE)?state:!state;
  gpio_set_level(dev->conf.gpio,level);
  dev->state = state;
  dev->esp_timestamp = esp_timer_get_time();
  dev->sen.esp_timestamp = dev->esp_timestamp;
  return ESP_OK;
  // dev->sen.outs[0].processed = state;  TODO: maybe state is only changed when gpio_get_level
}
// esp_err_t switch_sen_iot_sen_measurement(void *dev) {
//   //TODO: get latest sensor data
//   return ESP_OK;
// }
