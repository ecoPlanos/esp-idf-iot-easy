/*
 * Copyright (c) 2023 ecoPlanos <geral@ecoplanos.pt>
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


#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>
#include <string.h>
#include "bmv60x.h"

#define BMV60X_DATA_FRAME_LENGTH  30 //maximum VED line characters
#define VED_START_BYTE_1 0x0D
#define VED_START_BYTE_2 0x0A
#define VED_SEPARATOR_BYTE 0x09

static const char *TAG = "bmv60x";

static bmv60x_decode_state_t decode_state=BMV60X_DECODE_READ_STATE_WAIT;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) {ESP_ERROR_CHECK_WITHOUT_ABORT(__); return __;} } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static inline esp_err_t send_cmd_nolock(bmv60x_t *dev, uint8_t cmd[]) {
    ESP_LOGD(TAG, "Sending cmd %02x %02x %02x %02x %02x %02x %02x...", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
    int wr_len = uart_write_bytes(dev->uart_dev.port, cmd, 7);
    if(wr_len < 7) return ESP_FAIL;
    return ESP_OK;
}

static inline esp_err_t read_res_nolock(bmv60x_t *dev, bmv60x_raw_data_t *res) {
  char data_label[BMV60X_DATA_LABEL_LENGTH];
  char data_val[BMV60X_DATA_VAL_LENGTH];
  uint8_t c;
  size_t available=0, start_idx=0, in_idx=0, data_idx=0, i=0;
  uint8_t checksum=0;
  esp_err_t err=ESP_OK;
  uint8_t err_cnt=0;
  bool label_detect=false;
  bmv60x_label_type_t label_type=BMV60X_LABEL_TYPE_UNDEF;
  uint8_t label_idx=0;
  if(decode_state!=BMV60X_DECODE_READ_STATE_WAIT){
    ESP_LOGE(TAG, "Other process is reading device right now...");
    return ESP_ERR_INVALID_STATE;
  }
  decode_state=BMV60X_DECODE_READ_STATE_START1;
  memset(data_label,'\0',BMV60X_DATA_LABEL_LENGTH);
  memset(data_val,'\0',BMV60X_DATA_VAL_LENGTH);
  UART_DEV_CHECK(&dev->uart_dev, uart_get_buffered_data_len(dev->uart_dev.port, &available));
  if(available<BMV60X_DATA_LABEL_LENGTH){
    ESP_LOGE(TAG, "Not enough data on RX buffer. Available: %d", available);
    // ESP_LOGI(TAG, "Cleaning RX buffer due to wrong length");
    // uart_flush(dev->uart_dev.port);
    err_cnt++;
    if(err_cnt > 10){
      ESP_LOGE(TAG,"Many errors on UART read");
      return ESP_FAIL;
    }
  }
  ESP_LOGD(TAG, "Rx buffer bytes available: %d", available);
  dev->meas_start_time=esp_timer_get_time();
  while(1) {
    while(uart_dev_read(dev->uart_dev.port, &c, 1, 200) != ESP_OK){
      err_cnt++;
      ESP_LOGE(TAG, "Couldn't get byte from UART!");
      err=uart_get_buffered_data_len(dev->uart_dev.port, &available);
      if(err!=ESP_OK){
        ESP_LOGE(TAG, "Couldn't get available bytes!");
        ESP_LOGI(TAG, "Assuming UART is broken and restarting...");
        // ESP_ERROR_CHECK_WITHOUT_ABORT(bmv60x_init(dev));
        if(err_cnt > 10){
          ESP_LOGE(TAG,"Many errors on UART read");
          return ESP_FAIL;
        }
      }
      ESP_LOGD(TAG, "Rx buffer bytes available: %d", available);
      if(!available) break;
    }
    if((err!=ESP_OK) && (!available)){
      ESP_LOGI(TAG,"End of serial read");
      break;
    } 

    ESP_LOGV(TAG, "UART char: 0x%x",c);
    ESP_LOGV(TAG, "in_idx: %u",in_idx);
    checksum+=c;
    ESP_LOGV(TAG,"CHECKSUM: %u", checksum);
    switch(decode_state){
      case BMV60X_DECODE_READ_STATE_START1:
        if (c==VED_START_BYTE_1){
          checksum=VED_START_BYTE_1;
          decode_state=BMV60X_DECODE_READ_STATE_START2;
          ESP_LOGD(TAG, "Found start high byte");
        }
        break;
      case BMV60X_DECODE_READ_STATE_START2:
        if (c==VED_START_BYTE_2){
          in_idx=0;
          decode_state=BMV60X_DECODE_READ_STATE_LABEL;
          ESP_LOGD(TAG, "Found start low byte");
        }else{
          decode_state=BMV60X_DECODE_READ_STATE_START1;
          ESP_LOGE(TAG, "Found wrong byte after start byte 1: 0x%x",c);
        }
        break;
      case BMV60X_DECODE_READ_STATE_LABEL:
        if(c==VED_SEPARATOR_BYTE){
          for(i=0;i<BMV60X_OUT_MAX; i++){
            if(strcmp(data_label, bmv60x_out_ved_labels[i]) == 0){
              label_detect=true;
              label_type=BMV60X_LABEL_TYPE_OUT;
              label_idx=i;
            }
          }
          if(!label_detect){
            for(i=0;i<BMV60X_INF_MAX;i++){
              if(strcmp(data_label, bmv60x_info_ved_labels[i]) == 0){
                label_detect=true;
                label_type=BMV60X_LABEL_TYPE_INFO;
                label_idx=i;
              }
            }
          }
          if(!label_detect){
            ESP_LOGE(TAG, "Label \"%s\" not found on known labels!",data_label);
            return ESP_FAIL;
          }
          in_idx=0;
          decode_state=BMV60X_DECODE_READ_STATE_DATA;
          break;
        }
        data_label[in_idx]=c;
        in_idx++;
        break;
      case BMV60X_DECODE_READ_STATE_DATA:
        if((label_type==BMV60X_LABEL_TYPE_INFO) && (label_idx==BMV60X_INF_CHECKSUM)){
          ESP_LOGD(TAG,"Received checksum: 0x%x",c);
          if(checksum==0){
            ESP_LOGD(TAG,"Success decoding block");
            // return ESP_OK;
          } else {
            ESP_LOGE(TAG, "Checksum error! Sum val: %d", checksum);
            // return ESP_FAIL;
          }
          decode_state=BMV60X_DECODE_READ_STATE_START1;
          break;
        }
        if(c==VED_START_BYTE_1){
          switch(label_type){
            case BMV60X_LABEL_TYPE_INFO:
              switch(label_idx){
                case BMV60X_INF_BMV:
                  if(dev->sen.info.model[0]=='\0' || dev->sen.info.model[0]!=data_val[0])
                    // strncpy(dev->sen.info.model, data_val, CONFIG_SENSOR_HANDLER_SEN_MODEL_LEN);
                    strcpy(dev->sen.info.model, data_val);
                    ESP_LOGI(TAG,"BMV model description: %s",data_val);
                  break;
                case BMV60X_INF_FW:
                  if(dev->sen.info.model[0]=='\0' || dev->sen.info.model[0]!=data_val[0])
                    // strncpy(dev->sen.info.fw, data_val, CONFIG_SENSOR_HANDLER_SEN_FW_LEN);
                    strcpy(dev->sen.info.fw, data_val);
                    ESP_LOGI(TAG,"BMV firmware version: %s",data_val);
                  break;
                case BMV60X_INF_CHECKSUM:
                  ESP_LOGD(TAG,"Received checksum: 0x%x",data_val[0]);
                  if(checksum==0){
                    ESP_LOGD(TAG,"Success decoding block");
                  } else {
                    ESP_LOGE(TAG, "Checksum error! Sum val: %d", checksum);
                  }
                  break;
                default:
                  ESP_LOGE(TAG, "Invalid info label!");
                  break;
              }
              break;
            case BMV60X_LABEL_TYPE_OUT:
              switch(label_idx){
                case BMV60X_OUT_V:
                  res->v = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mv)",bmv60x_out_ved_labels[label_idx],res->v);
                  break;
                case BMV60X_OUT_VS:
                  res->vs = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mv)",bmv60x_out_ved_labels[label_idx],res->vs);
                  break;
                case BMV60X_OUT_I:
                  res->i = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mA)",bmv60x_out_ved_labels[label_idx],res->i);
                  break;
                case BMV60X_OUT_CE:
                  res->ce = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mAh)",bmv60x_out_ved_labels[label_idx],res->ce);
                  break;
                case BMV60X_OUT_SOC:
                  res->soc = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (%%)",bmv60x_out_ved_labels[label_idx],res->soc);
                  break;
                case BMV60X_OUT_TIG:
                  res->tig = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (min)",bmv60x_out_ved_labels[label_idx],res->tig);
                  break;
                case BMV60X_OUT_Alarm:
                  res->alarm = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (bool)",bmv60x_out_ved_labels[label_idx],res->alarm);
                  break;
                case BMV60X_OUT_Relay:
                  res->relay = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (bool)",bmv60x_out_ved_labels[label_idx],res->relay);
                  break;
                case BMV60X_OUT_AR:
                  res->ar = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (reason code)",bmv60x_out_ved_labels[label_idx],res->ar);
                  break;
                case BMV60X_OUT_H1:
                  res->h1 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mAh)",bmv60x_out_ved_labels[label_idx],res->h1);
                  break;
                case BMV60X_OUT_H2:
                  res->h2 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mAh)",bmv60x_out_ved_labels[label_idx],res->h2);
                  break;
                case BMV60X_OUT_H3:
                  res->h3 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mAh)",bmv60x_out_ved_labels[label_idx],res->h3);
                  break;
                case BMV60X_OUT_H4:
                  res->h4 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  cycles",bmv60x_out_ved_labels[label_idx],res->h4);
                  break;
                case BMV60X_OUT_H5:
                  res->h5 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  discharges",bmv60x_out_ved_labels[label_idx],res->h5);
                  break;
                case BMV60X_OUT_H6:
                  res->h6 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mAh)",bmv60x_out_ved_labels[label_idx],res->h6);
                  break;
                case BMV60X_OUT_H7:
                  res->h7 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mV)",bmv60x_out_ved_labels[label_idx],res->h7);
                  break;
                case BMV60X_OUT_H8:
                  res->h8 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mV)",bmv60x_out_ved_labels[label_idx],res->h8);
                  break;
                case BMV60X_OUT_H9:
                  res->h9 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (sec)",bmv60x_out_ved_labels[label_idx],res->h9);
                  break;
                case BMV60X_OUT_H10:
                  res->h10 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  synchronizations",bmv60x_out_ved_labels[label_idx],res->h10);
                  break;
                case BMV60X_OUT_H11:
                  res->h11 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (nr)",bmv60x_out_ved_labels[label_idx],res->h11);
                  break;
                case BMV60X_OUT_H12:
                  res->h12 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (nr)",bmv60x_out_ved_labels[label_idx],res->h12);
                  break;
                case BMV60X_OUT_H13:
                  res->h13 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (nr)",bmv60x_out_ved_labels[label_idx],res->h13);
                  break;
                case BMV60X_OUT_H14:
                  res->h14 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d  (nr)",bmv60x_out_ved_labels[label_idx],res->h14);
                  break;
                case BMV60X_OUT_H15:
                  res->h15 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mV)",bmv60x_out_ved_labels[label_idx],res->h15);
                  break;
                case BMV60X_OUT_H16:
                  res->h16 = atoi(data_val);
                  ESP_LOGD(TAG,"Received %s data: %d (mV)",bmv60x_out_ved_labels[label_idx],res->h16);
                  break;
                default:
                  ESP_LOGE(TAG, "Invalid data label!");
                  break;
              }
              break;
            case BMV60X_LABEL_TYPE_UNDEF:
              ESP_LOGE(TAG,"Invalid label type is undefined!");
              break;
            default:
              ESP_LOGE(TAG,"Invalid label type!");
              break;
          }
          decode_state=BMV60X_DECODE_READ_STATE_START1;
        }else{
          data_val[in_idx]=c;
          in_idx++;
        }
        break;
      default:
        ESP_LOGE(TAG, "Invalid parsing state! %u",decode_state);
        return ESP_ERR_INVALID_STATE;
        break;
    }
  }
  decode_state=BMV60X_DECODE_READ_STATE_WAIT;
  return ESP_OK;
}

static esp_err_t read_res(bmv60x_t *dev, bmv60x_raw_data_t *res) {
  UART_DEV_TAKE_MUTEX(&dev->uart_dev);
  UART_DEV_CHECK(&dev->uart_dev, read_res_nolock(dev, res));
  UART_DEV_GIVE_MUTEX(&dev->uart_dev);

  return ESP_OK;
}

static esp_err_t exec_cmd(bmv60x_t *dev, uint8_t cmd[], size_t delay_ticks) {
  UART_DEV_TAKE_MUTEX(&dev->uart_dev);
  UART_DEV_CHECK(&dev->uart_dev, send_cmd_nolock(dev, cmd));
  if (delay_ticks) vTaskDelay(delay_ticks);
  UART_DEV_GIVE_MUTEX(&dev->uart_dev);

  return ESP_OK;
}

static inline bool is_measuring(bmv60x_t *dev) {
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;
    return (elapsed < dev->sen.conf.delay_start_get_us);
}

static inline void print_raw_values(bmv60x_t *dev) {
  ESP_LOGD(TAG, "esp_timestamp: %llu",dev->sen.esp_timestamp);
  ESP_LOGD(TAG, "Batt Voltage: %u",dev->sen.outs[BMV60X_OUT_V].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_VS].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_I].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_CE].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_SOC].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_TIG].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_Alarm].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_Relay].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_AR].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_H1].m_raw);
  ESP_LOGD(TAG, ": %d",dev->sen.outs[BMV60X_OUT_H2].m_raw);
}

static void uart_event_task(void *param) {
    bmv60x_t *bmv60x = param;
    uart_event_t event;
    char b;
    while (true) {
      if (xQueueReceive(bmv60x->uart_dev.queue, &event, portMAX_DELAY)) {
        ESP_LOGD(TAG, "Received UART event: %d",event.type);
        switch (event.type) {
          case UART_BREAK:
          case UART_PATTERN_DET:
            while(uart_dev_read(bmv60x->uart_dev.port, &b, 1, 200)==ESP_OK){
              ESP_LOGD(TAG, "PATTERN DETECT: %c",b);
            }
            break;
          case UART_DATA:
            while(uart_dev_read(bmv60x->uart_dev.port, &b, 1, 200)==ESP_OK){
              ESP_LOGD(TAG, "SERIAL DATA: %c",b);
            }
            break;
          default:
            break;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t bmv60x_init_desc(bmv60x_t *dev, uart_port_t port, gpio_num_t tx_gpio, \
                            gpio_num_t rx_gpio, int tx_buffer_size, int rx_buffer_size, \
                            int queue_size, uart_sclk_t source_clk, uint16_t sen_id) {
    CHECK_ARG(dev);
#ifdef CONFIG_BMV60X_ACTIVE_MODE
    dev->config.data_mode = BMV60X_DATA_MODE_ACTIVE;
#else
    dev->config.data_mode = BMV60X_DATA_MODE_PASSIVE;
#endif
    dev->status.sleep_mode = BMV60X_SLEEP_MODE_SLEEP;
    dev->uart_dev.port = port;
    dev->uart_dev.tx_io_num = tx_gpio;
    dev->uart_dev.rx_io_num = rx_gpio;
    dev->uart_dev.tx_buffer_size = tx_buffer_size;
    dev->uart_dev.rx_buffer_size = rx_buffer_size;
    dev->uart_dev.queue_size = queue_size;
    // dev->uart_dev.queue = NULL;
    // dev->uart_dev.intr_alloc_flags = intr_alloc_flags;
    dev->uart_dev.cfg.baud_rate = 19200;

    dev->uart_dev.cfg.data_bits = UART_DATA_8_BITS;
    dev->uart_dev.cfg.parity = UART_PARITY_DISABLE;
    dev->uart_dev.cfg.stop_bits = UART_STOP_BITS_1;
    dev->uart_dev.cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    dev->uart_dev.cfg.source_clk = source_clk;

    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,BMV60X_OUT_MAX);
    strncpy(dev->sen.info.name, "BMV60X\0", 7);
    // dev->sen.conf.delay_after_awake_us = (uint32_t) (CONFIG_BMV60X_AFTER_AWAKE_DELAY_MS*1000);
    dev->sen.conf.delay_after_awake_us = 0;
    dev->sen.conf.time_to_adjust_us = 0;
    dev->sen.conf.delay_start_get_us = 1420000;
    dev->sen.info.lib_id = SEN_BMV60X_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.info.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.conf.min_period_us = 420420;
    dev->sen.info.out_nr = BMV60X_OUT_MAX;
    dev->sen.conf.trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.conf.period_ms=CONFIG_BMV60X_DEFAULT_PERIOD_MS;
    dev->sen.dev=dev;
    dev->sen.reset=bmv60x_iot_sen_reset;
    dev->sen.reinit=bmv60x_iot_sen_reinit;
    dev->sen.start_measurement=bmv60x_iot_sen_start_measurement;
    dev->sen.get_data=bmv60x_iot_sen_get_data;
    dev->sen.awake=bmv60x_iot_sen_sleep_mode_awake;
    dev->sen.sleep=bmv60x_iot_sen_sleep_mode_sleep;

    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;
    dev->sen.status.delay_m_us = 0;

    dev->sen.outs[BMV60X_OUT_V].out_id=BMV60X_OUT_V;
    dev->sen.outs[BMV60X_OUT_V].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_V].m_raw=0;
    dev->sen.outs[BMV60X_OUT_V].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_V].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_VS].out_id=BMV60X_OUT_VS;
    dev->sen.outs[BMV60X_OUT_VS].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_VS].m_raw=0;
    dev->sen.outs[BMV60X_OUT_VS].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_VS].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_I].out_id=BMV60X_OUT_I;
    dev->sen.outs[BMV60X_OUT_I].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_I].m_raw=0;
    dev->sen.outs[BMV60X_OUT_I].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_I].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_CE].out_id=BMV60X_OUT_CE;
    dev->sen.outs[BMV60X_OUT_CE].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_CE].m_raw=0;
    dev->sen.outs[BMV60X_OUT_CE].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_CE].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_SOC].out_id=BMV60X_OUT_SOC;
    dev->sen.outs[BMV60X_OUT_SOC].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_SOC].m_raw=0;
    dev->sen.outs[BMV60X_OUT_SOC].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_SOC].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_TIG].out_id=BMV60X_OUT_TIG;
    dev->sen.outs[BMV60X_OUT_TIG].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_TIG].m_raw=0;
    dev->sen.outs[BMV60X_OUT_TIG].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_TIG].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_Alarm].out_id=BMV60X_OUT_Alarm;
    dev->sen.outs[BMV60X_OUT_Alarm].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_Alarm].m_raw=0;
    dev->sen.outs[BMV60X_OUT_Alarm].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_Alarm].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_Relay].out_id=BMV60X_OUT_Relay;
    dev->sen.outs[BMV60X_OUT_Relay].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_Relay].m_raw=0;
    dev->sen.outs[BMV60X_OUT_Relay].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_Relay].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_AR].out_id=BMV60X_OUT_AR;
    dev->sen.outs[BMV60X_OUT_AR].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_AR].m_raw=0;
    dev->sen.outs[BMV60X_OUT_AR].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_AR].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H1].out_id=BMV60X_OUT_H1;
    dev->sen.outs[BMV60X_OUT_H1].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H1].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H1].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H1].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H2].out_id=BMV60X_OUT_H2;
    dev->sen.outs[BMV60X_OUT_H2].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H2].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H2].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H2].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H3].out_id=BMV60X_OUT_H3;
    dev->sen.outs[BMV60X_OUT_H3].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H3].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H3].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H3].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H4].out_id=BMV60X_OUT_H4;
    dev->sen.outs[BMV60X_OUT_H4].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H4].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H4].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H4].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H5].out_id=BMV60X_OUT_H5;
    dev->sen.outs[BMV60X_OUT_H5].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H5].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H5].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H5].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H6].out_id=BMV60X_OUT_H6;
    dev->sen.outs[BMV60X_OUT_H6].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H6].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H6].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H6].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H7].out_id=BMV60X_OUT_H7;
    dev->sen.outs[BMV60X_OUT_H7].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H7].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H7].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H7].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H8].out_id=BMV60X_OUT_H8;
    dev->sen.outs[BMV60X_OUT_H8].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H8].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H8].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H8].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H9].out_id=BMV60X_OUT_H9;
    dev->sen.outs[BMV60X_OUT_H9].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H9].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H9].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H9].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H10].out_id=BMV60X_OUT_H10;
    dev->sen.outs[BMV60X_OUT_H10].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H10].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H10].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H10].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H11].out_id=BMV60X_OUT_H11;
    dev->sen.outs[BMV60X_OUT_H11].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H11].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H11].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H11].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H12].out_id=BMV60X_OUT_H12;
    dev->sen.outs[BMV60X_OUT_H12].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H12].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H12].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H12].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H13].out_id=BMV60X_OUT_H13;
    dev->sen.outs[BMV60X_OUT_H13].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H13].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H13].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H13].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H14].out_id=BMV60X_OUT_H14;
    dev->sen.outs[BMV60X_OUT_H14].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H14].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H14].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H14].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H15].out_id=BMV60X_OUT_H15;
    dev->sen.outs[BMV60X_OUT_H15].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H15].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H15].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H15].conf.srate = 0;

    dev->sen.outs[BMV60X_OUT_H16].out_id=BMV60X_OUT_H16;
    dev->sen.outs[BMV60X_OUT_H16].out_val_type=SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[BMV60X_OUT_H16].m_raw=0;
    dev->sen.outs[BMV60X_OUT_H16].processed=0.0;
    // dev->sen.outs[BMV60X_OUT_H16].conf.srate = 0;

    if(CONFIG_BMV60X_MAX_MODEL_DESC > CONFIG_SENSOR_HANDLER_SEN_MODEL_LEN ){
      ESP_LOGE(TAG, "\"CONFIG_BMV60X_MAX_MODEL_DESC\" can't be greater than \"CONFIG_SENSOR_HANDLER_SEN_MODEL_LEN\"" );
    }
    memset(dev->info.bmv,'\0',CONFIG_SENSOR_HANDLER_SEN_MODEL_LEN);
    dev->info.fw=0;

    dev->sen.conf.srate = 0;

    return uart_dev_create_mutex(&dev->uart_dev);
}

esp_err_t bmv60x_free_desc(bmv60x_t *dev) {
    CHECK_ARG(dev);

    return uart_dev_delete_mutex(&dev->uart_dev);
}

esp_err_t bmv60x_init(bmv60x_t *dev) {
  size_t available=0;
  char senid_str[4];
  CHECK_ARG(dev);
  CHECK(uart_enable_pattern_det_baud_intr(dev->uart_dev.port, VED_START_BYTE_1, 1, CONFIG_BMV60X_MESSAGE_TIMEOUT, 0, 0));
  char task_name[10];
  memset(task_name,'\0',10);
  memset(senid_str,'\0',4);
  sprintf(senid_str, "%d", dev->sen.info.sen_id);
  strcat(task_name,senid_str);
  xTaskCreatePinnedToCore(
    uart_event_task,
    task_name,
    configMINIMAL_STACK_SIZE + 512,
    dev,
    CONFIG_PTHREAD_TASK_PRIO_DEFAULT,
    &dev->event_task,
    xPortGetCoreID());
  /* Set pattern interrupt, used to detect the end of a line */
  // uart_enable_pattern_det_baud_intr(dev->uart_dev.port, VED_START_BYTE_1, 1, 9, 0, 0);
  /* Set pattern queue size */
  // uart_pattern_queue_reset(dev->uart_dev.port, dev->uart_dev.queue_size);

  uint64_t init_start=esp_timer_get_time();
  vTaskDelay(pdMS_TO_TICKS(100));

  while(1){
    UART_DEV_CHECK(&dev->uart_dev, uart_get_buffered_data_len(dev->uart_dev.port, &available));
    if((available) || ((esp_timer_get_time()-init_start)>dev->sen.conf.delay_start_get_us)) break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGD(TAG, "Available Rx buffer bytes: %d", available);
  ESP_LOGD(TAG, "time untill get data: %llu", (esp_timer_get_time()-init_start));
  // CHECK(bmv60x_set_sleep_mode(dev, BMV60X_SLEEP_MODE_SLEEP));
  if(available){
    dev->sen.status.status_code=SEN_STATUS_OK;
    ESP_LOGI(TAG, "Sensor initialized!");
    return ESP_OK;
  }
  return ESP_FAIL;
}

esp_err_t bmv60x_reset(bmv60x_t *dev) {
  CHECK_ARG(dev);
  ESP_LOGI(TAG, "Sensor is restarting...");
  uart_flush(dev->uart_dev.port);
  return ESP_OK;
}

esp_err_t bmv60x_measure(bmv60x_t *dev, bmv60x_raw_data_t *raw) {
  // esp_err_t ret;
  ESP_LOGD(TAG, "bmv60x sen measurement");
  // CHECK_ARG(dev && (pm1_0_con_unit || pm2_5_con_unit || pm10_con_unit || pm1_0_con_unit_atmosphe || pm2_5_con_unit_atmosphe || con_unit_atmosphe || particle_nr_0_3_um || particle_nr_0_5_um || particle_nr_1_0_um || particle_nr_2_5_um || particle_nr_5_0_um || particle_nr_10_um));
  CHECK_ARG(dev && raw);
  // CHECK_ARG(dev);
  CHECK(bmv60x_start_measurement(dev));
  // uint8_t cmd[7];
  // CHECK(get_meas_cmd(cmd));
  // CHECK(exec_cmd(dev,cmd, 0));
  vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  while(is_measuring(dev));
  CHECK(bmv60x_get_raw_data(dev,raw));
  // if(dev->conf.delay_after_awake_ms)
  //   bmv60x_set_sleep_mode(dev, BMV60X_SLEEP_MODE_SLEEP);
  return bmv60x_compute_values(dev, &raw);
}

esp_err_t bmv60x_set_data_mode(bmv60x_t *dev, bmv60x_mode_type_t data_mode) {
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "Setting data mode: %u",data_mode);
  dev->config.data_mode = data_mode;
  return ESP_OK;
}

esp_err_t bmv60x_set_sleep_mode(bmv60x_t *dev, bmv60x_sleep_type_t sleep_mode) {
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "bmv60x_set_sleep_mode");
  ESP_LOGD(TAG, "Initial sleep mode: %u", dev->status.sleep_mode);
  if((sleep_mode!=BMV60X_SLEEP_MODE_SLEEP) && (sleep_mode!=BMV60X_SLEEP_MODE_AWAKE)){
    ESP_LOGE(TAG, "Ivalid sleep mode!");
    return ESP_ERR_INVALID_STATE;
  }
  dev->status.sleep_mode = sleep_mode;
  ESP_LOGD(TAG, "Current sleep mode: %u", dev->status.sleep_mode);
  return ESP_OK;
}

esp_err_t bmv60x_toggle_sleep_mode(bmv60x_t *dev) {
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "bmv60x_toggle_sleep_mode");
  if(dev->status.sleep_mode==BMV60X_SLEEP_MODE_SLEEP) {
    CHECK(bmv60x_set_sleep_mode(dev, BMV60X_SLEEP_MODE_AWAKE));
  }
  else {
    CHECK(bmv60x_set_sleep_mode(dev, BMV60X_SLEEP_MODE_SLEEP));
  }

  return ESP_OK;
}

esp_err_t bmv60x_start_measurement(bmv60x_t *dev) {
  CHECK_ARG(dev);
  esp_err_t ret;
  uint8_t cmd[7];
  ESP_LOGD(TAG, "Start measurement");

  if (is_measuring(dev)) {
    ESP_LOGE(TAG, "Measurement is still running");
    return ESP_ERR_INVALID_STATE;
  }

  if(dev->status.sleep_mode == BMV60X_SLEEP_MODE_SLEEP) {
    CHECK(bmv60x_set_sleep_mode(dev, BMV60X_SLEEP_MODE_AWAKE));
  }
  return ESP_OK;
}

size_t bmv60x_get_measurement_duration(bmv60x_t *dev) {
    CHECK_ARG(dev);
    return esp_timer_get_time()-dev->meas_start_time;
}

esp_err_t bmv60x_get_raw_data(bmv60x_t *dev, bmv60x_raw_data_t *raw) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "bmv60x_get_raw_data");
    CHECK(read_res(dev, raw));
    dev->sen.outs[BMV60X_OUT_V].m_raw=raw->v;
    dev->sen.outs[BMV60X_OUT_VS].m_raw=raw->vs;
    dev->sen.outs[BMV60X_OUT_I].m_raw=raw->i;
    dev->sen.outs[BMV60X_OUT_CE].m_raw=raw->ce;
    dev->sen.outs[BMV60X_OUT_SOC].m_raw=raw->soc;
    dev->sen.outs[BMV60X_OUT_TIG].m_raw=raw->tig;
    dev->sen.outs[BMV60X_OUT_Alarm].m_raw=raw->alarm;
    dev->sen.outs[BMV60X_OUT_Relay].m_raw=raw->relay;
    dev->sen.outs[BMV60X_OUT_AR].m_raw=raw->ar;
    dev->sen.outs[BMV60X_OUT_H1].m_raw=raw->h1;
    dev->sen.outs[BMV60X_OUT_H2].m_raw=raw->h2;
    dev->sen.outs[BMV60X_OUT_H3].m_raw=raw->h3;
    dev->sen.outs[BMV60X_OUT_H4].m_raw=raw->h4;
    dev->sen.outs[BMV60X_OUT_H5].m_raw=raw->h5;
    dev->sen.outs[BMV60X_OUT_H6].m_raw=raw->h6;
    dev->sen.outs[BMV60X_OUT_H7].m_raw=raw->h7;
    dev->sen.outs[BMV60X_OUT_H8].m_raw=raw->h8;
    dev->sen.outs[BMV60X_OUT_H9].m_raw=raw->h9;
    dev->sen.outs[BMV60X_OUT_H10].m_raw=raw->h10;
    dev->sen.outs[BMV60X_OUT_H11].m_raw=raw->h11;
    dev->sen.outs[BMV60X_OUT_H12].m_raw=raw->h12;
    dev->sen.outs[BMV60X_OUT_H13].m_raw=raw->h13;
    dev->sen.outs[BMV60X_OUT_H14].m_raw=raw->h14;
    dev->sen.outs[BMV60X_OUT_H15].m_raw=raw->h15;
    dev->sen.outs[BMV60X_OUT_H16].m_raw=raw->h16;
    dev->sen.esp_timestamp=dev->meas_start_time;
    return ESP_OK;
}

esp_err_t bmv60x_compute_values(bmv60x_t *dev, bmv60x_raw_data_t *raw) {
  bmv60x_get_raw_data(dev, raw);
  dev->sen.outs[BMV60X_OUT_V].processed=(float)raw->v/1000.0;
  dev->sen.outs[BMV60X_OUT_VS].processed=(float)raw->vs/1000.0;
  dev->sen.outs[BMV60X_OUT_I].processed=(float)raw->i/1000.0;
  dev->sen.outs[BMV60X_OUT_CE].processed=(float)raw->ce/1000.0;
  dev->sen.outs[BMV60X_OUT_SOC].processed=raw->soc;
  dev->sen.outs[BMV60X_OUT_TIG].processed=raw->tig;
  dev->sen.outs[BMV60X_OUT_Alarm].processed=raw->alarm;
  dev->sen.outs[BMV60X_OUT_Relay].processed=raw->relay;
  dev->sen.outs[BMV60X_OUT_AR].processed=raw->ar;
  dev->sen.outs[BMV60X_OUT_H1].processed=(float)raw->h1/1000.0;
  dev->sen.outs[BMV60X_OUT_H2].processed=(float)raw->h2/1000.0;
  dev->sen.outs[BMV60X_OUT_H3].processed=(float)raw->h3/1000.0;
  dev->sen.outs[BMV60X_OUT_H4].processed=raw->h4;
  dev->sen.outs[BMV60X_OUT_H5].processed=raw->h5;
  dev->sen.outs[BMV60X_OUT_H6].processed=(float)raw->h6/1000.0;
  dev->sen.outs[BMV60X_OUT_H7].processed=(float)raw->h7/1000.0;
  dev->sen.outs[BMV60X_OUT_H8].processed=(float)raw->h8/1000.0;
  dev->sen.outs[BMV60X_OUT_H9].processed=raw->h9;
  dev->sen.outs[BMV60X_OUT_H10].processed=raw->h10;
  dev->sen.outs[BMV60X_OUT_H11].processed=raw->h11;
  dev->sen.outs[BMV60X_OUT_H12].processed=raw->h12;
  dev->sen.outs[BMV60X_OUT_H13].processed=raw->h13;
  dev->sen.outs[BMV60X_OUT_H14].processed=raw->h14;
  dev->sen.outs[BMV60X_OUT_H15].processed=(float)raw->h15/1000.0;
  dev->sen.outs[BMV60X_OUT_H16].processed=(float)raw->h16/1000.0;
  dev->sen.esp_timestamp=dev->meas_start_time;
  return ESP_OK;
}

esp_err_t bmv60x_get_results(bmv60x_t *dev) {
  CHECK_ARG(dev);
  bmv60x_raw_data_t raw;
  ESP_LOGD(TAG, "bmv60x_get_results");
  CHECK(bmv60x_get_raw_data(dev, &raw));
  return bmv60x_compute_values(dev, &raw);
}

esp_err_t bmv60x_iot_sen_start_measurement(void *dev) {
  return bmv60x_start_measurement((bmv60x_t *)dev);
}

esp_err_t bmv60x_iot_sen_get_data(void *dev) {
  esp_err_t ret;
  bmv60x_raw_data_t raw;
  // CHECK(bmv60x_get_raw_data(dev,&raw));
  // return bmv60x_compute_values(dev, &raw);
  return bmv60x_get_results((bmv60x_t *)dev);
  // return bmv60x_measure((bmv60x_t *)dev, &raw);
  // return ESP_OK;
  // return bmv60x_measure((bmv60x_t *)dev, &raw);
}

esp_err_t bmv60x_iot_sen_toggle_sleep_mode(void *dev) {
  return bmv60x_toggle_sleep_mode((bmv60x_t *)dev);
}

esp_err_t bmv60x_iot_sen_sleep_mode_awake(void *dev) {
  return bmv60x_set_sleep_mode((bmv60x_t *)dev, BMV60X_SLEEP_MODE_AWAKE);
}

esp_err_t bmv60x_iot_sen_sleep_mode_sleep(void *dev) {
  return bmv60x_set_sleep_mode((bmv60x_t *)dev, BMV60X_SLEEP_MODE_SLEEP);
}

esp_err_t bmv60x_iot_sen_reset(void *dev) {
  return bmv60x_reset((bmv60x_t *)dev);
}

esp_err_t bmv60x_iot_sen_reinit(void *dev) {
  return bmv60x_init((bmv60x_t *)dev);
}
