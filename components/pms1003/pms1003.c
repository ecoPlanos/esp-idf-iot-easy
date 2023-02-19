/*
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
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
 * @file pms1003.c
 *
 * ESP-IDF driver for Sensirion PMS1003 digital temperature and humidity sensor
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
#include "pms1003.h"

#define PMS1003_DATA_FRAME_LENGTH  28 //(2*13 data bytes + 2 check bytes)
#define ANSWER_LENGTH 32 //Sensor reply with 32 byte length message

static const char *TAG = "pms1003";

typedef enum {
  START_BYTE_1 = 0x42,
  START_BYTE_2 = 0x4d,
  START = 0x424d
} pms1003_start_t;

typedef enum {
  CMD_READ_PASSIVE_MODE = 0xe2,
  CMD_CHANGE_MODE =       0xe1,
  CMD_CHANGE_SLEEP =      0xe4
} pms1003_commands_t;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) {ESP_ERROR_CHECK_WITHOUT_ABORT(__); return __;} } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t check_code(uint8_t data[], uint16_t len) {
    uint16_t sum = len+START_BYTE_1+START_BYTE_2;

    ESP_LOGD(TAG, "sum: 0x%4x",sum);
    for (size_t i = 0; i < len-2; i++) sum += data[i];

    ESP_LOGD(TAG, "length: %u",len);
    ESP_LOGD(TAG, "length[0]: 0x%2x",(len>>8)&0xff);
    ESP_LOGD(TAG, "length[1]: 0x%2x",(len)&0xff);
    ESP_LOGD(TAG, "length[0]: 0x%2x",(sum>>8)&0xff);
    ESP_LOGD(TAG, "length[1]: 0x%2x",(sum)&0xff);
    ESP_LOGD(TAG, "data[len-2]: 0x%4x",data[len-2]);
    ESP_LOGD(TAG, "data[len-1]: 0x%4x",data[len-1]);
    if (((data[len-2]<<8)|data[len-1]) != sum) {
        ESP_LOGE(TAG, "Invalid CRC");
        ESP_LOGD(TAG, "received sum: 0x%4x",((data[len-2]<<8)|data[len-1]));
        ESP_LOGD(TAG, "sum: 0x%4x",sum);
        return ESP_ERR_INVALID_CRC;
    }
  return ESP_OK;
}

static inline esp_err_t get_meas_cmd(uint8_t cmd[]) {
  if(!cmd) return ESP_ERR_INVALID_ARG;
  cmd[0] = START_BYTE_1;
  cmd[1] = START_BYTE_2;
  cmd[2] = CMD_READ_PASSIVE_MODE;
  cmd[3] = 0;
  cmd[4] = 0;
  uint16_t check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
  cmd[5] = (check>>8)&0xFF;
  cmd[6] = check&0xFF;
  ESP_LOGD(TAG, "Sum: %4x check_MSB: %2x check_LSB: %2x", check, cmd[5], cmd[6]);
  return ESP_OK;
}

static inline esp_err_t get_set_data_mode_cmd(uint8_t cmd[], pms1003_mode_type_t mode) {
  if(!cmd) return ESP_ERR_INVALID_ARG;
  cmd[0] = START_BYTE_1;
  cmd[1] = START_BYTE_2;
  cmd[2] = CMD_CHANGE_MODE;
  cmd[3] = 0;
  cmd[4] = mode;
  uint16_t check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
  cmd[5] = (check>>8)&0xFF;
  cmd[6] = check&0xFF;
  return ESP_OK;
}

static inline esp_err_t get_set_sleep_mode_cmd(uint8_t cmd[], pms1003_sleep_type_t sleep) {
  if(!cmd) return ESP_ERR_INVALID_ARG;
  cmd[0] = START_BYTE_1;
  cmd[1] = START_BYTE_2;
  cmd[2] = CMD_CHANGE_SLEEP;
  cmd[3] = 0;
  cmd[4] = sleep;
  uint16_t check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
  cmd[5] = (check>>8)&0xFF;
  cmd[6] = check&0xFF;
  return ESP_OK;
}

static inline esp_err_t send_cmd_nolock(pms1003_t *dev, uint8_t cmd[]) {
    ESP_LOGD(TAG, "Sending cmd %02x %02x %02x %02x %02x %02x %02x...", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
    int wr_len = uart_write_bytes(dev->uart_dev.port, cmd, 7);
    if(wr_len < 7) return ESP_FAIL;
    return ESP_OK;
}

static inline esp_err_t read_res_nolock(pms1003_t *dev, pms1003_raw_data_t *res) {
  uint8_t res8[PMS1003_DATA_FRAME_LENGTH];
  uint8_t in_buff[PMS1003_DATA_FRAME_LENGTH];
  char c;
  uint8_t frame_len=0;
  int read_len=0;
  size_t available=0, start_idx=0, in_idx=0, data_idx=0, i=0;
  pms1003_decode_state_t decode_state = PMS_DECODE_READ_STATE_START1;
  esp_err_t err;
  // bool start1 = false;
  // int read_len = uart_read_bytes(dev->uart_dev.port, res8, PMS1003_RAW_DATA_SIZE, 800 / portTICK_PERIOD_MS);
  // res[PMS1003_RAW_DATA_SIZE] = '\0';
  // CHECK(uart_dev_read(&dev->uart_dev, NULL, 0, res8, PMS1003_TOTAL_DATA_SIZE));
  UART_DEV_CHECK(&dev->uart_dev, uart_get_buffered_data_len(dev->uart_dev.port, &available));
  if(available<(PMS1003_DATA_FRAME_LENGTH+4)){
    ESP_LOGE(TAG, "Not enough data on RX buffer. Available: %d", available);
    // ESP_LOGI(TAG, "Cleaning RX buffer due to wrong length");
    // uart_flush(dev->uart_dev.port);
    return ESP_ERR_INVALID_SIZE;
  }
  ESP_LOGD(TAG, "Rx buffer bytes available: %d", available);
  while(1) {
    read_len = uart_read_bytes(dev->uart_dev.port, &c, 1, pdMS_TO_TICKS(200));
    if(read_len!=1){  //TODO: add timeout to break loop!
      ESP_LOGE(TAG, "Couldn't get byte from UART!");
      ESP_LOGD(TAG, "Received %u bytes",in_idx);
      err=uart_get_buffered_data_len(dev->uart_dev.port, &available);
      if(err!=ESP_OK){
        ESP_LOGE(TAG, "Couldn't get available bytes!");
        ESP_LOGI(TAG, "Assuming UART is broken and restarting...");
        // ESP_ERROR_CHECK_WITHOUT_ABORT(pms1003_init(dev));
      }
      ESP_LOGD(TAG, "Rx buffer bytes available: %d", available);
      return ESP_FAIL;
    } else {
      ESP_LOGV(TAG, "UART char: 0x%x",c);
      ESP_LOGV(TAG, "in_idx: %u",in_idx);
      in_buff[in_idx]=c;
      switch(decode_state){
        case PMS_DECODE_READ_STATE_START1:
        if (c==START_BYTE_1){
          decode_state=PMS_DECODE_READ_STATE_START2;
          in_idx=1;
          ESP_LOGD(TAG, "Found start high byte");
        }else{
          in_idx=0;
        }
        break;
        case PMS_DECODE_READ_STATE_START2:
        if (c==START_BYTE_2){
          decode_state=PMS_DECODE_READ_STATE_FRAME_LEN1;
          in_idx=2;
          ESP_LOGD(TAG, "Found start low byte");
        }else{
          decode_state=PMS_DECODE_READ_STATE_START1;
          in_idx=0;
        }
        break;
        case PMS_DECODE_READ_STATE_FRAME_LEN1:
        in_idx=3;
        frame_len=(c<<8)&0xff00;
        decode_state=PMS_DECODE_READ_STATE_FRAME_LEN2;
        break;
        case PMS_DECODE_READ_STATE_FRAME_LEN2:
        in_idx=3;
        frame_len|=c;
        if(frame_len == PMS1003_DATA_FRAME_LENGTH){
          decode_state=PMS_DECODE_READ_STATE_DATA;
          data_idx=0;
          ESP_LOGD(TAG, "Found correct length for data message.");
        }else{
          if((in_buff[in_idx-1]==START_BYTE_1) && (in_buff[in_idx]==START_BYTE_2)) {
            decode_state=PMS_DECODE_READ_STATE_FRAME_LEN1;
          }else{
            decode_state=PMS_DECODE_READ_STATE_START1;
          }
          in_idx=0;
        }
        break;
        case PMS_DECODE_READ_STATE_DATA:
        res8[data_idx]=c;
        data_idx++;
        if(data_idx==PMS1003_DATA_FRAME_LENGTH){
          if(check_code(res8, PMS1003_DATA_FRAME_LENGTH) != ESP_OK){
            data_idx=0;
            decode_state=PMS_DECODE_READ_STATE_START1;
            in_idx=0;
          // for(i=in_idx-PMS1003_DATA_FRAME_LENGTH+2;i<=in_idx;i++) {
          //   if(in_idx[i]==START_BYTE_1){
            //   if(in_idx[i]==START_BYTE_1){
            //     if(i+4 == PMS1003_DATA_FRAME_LENGTH){
            //       decode_state=PMS_DECODE_READ_STATE_START2;
            //     }
            //     if(res8[i+1]==START_BYTE_2){
            //       if(i+4 == PMS1003_DATA_FRAME_LENGTH){
            //         decode_state=PMS_DECODE_READ_STATE_FRAME_LEN1;
            //       }
            //       if(res8[i+2]==0x1){
            //         if(i+4 == PMS1003_DATA_FRAME_LENGTH){
            //           decode_state=PMS_DECODE_READ_STATE_FRAME_LEN2;
            //         }
            //         if(res8[i+3]==0xc){
            //           if(i+4 == PMS1003_DATA_FRAME_LENGTH) data_idx=0;
            //           else{
            //             memcpy(res8,res8+i+4,PMS1003_DATA_FRAME_LENGTH-i-1-4);
            //             data_idx=PMS1003_DATA_FRAME_LENGTH-i-1-4;
            //           }
            //           uint8_t j;
            //           for(j=0;j<PMS1003_DATA_FRAME_LENGTH;j++){
            //             ESP_LOGD(TAG, "checkFail res8[%u]: %u",j,res8[j]);
            //           }
            //           break;
            //         }
            //       }
            //     }
            //   }
            // }
          }else{
              res->pm1_0_con_unit = (res8[0]<<8)|res8[1];
              res->pm2_5_con_unit = (res8[2]<<8)|res8[3];
              res->pm10_con_unit = (res8[4]<<8)|res8[5];
              res->pm1_0_con_unit_atmosphe = (res8[6]<<8)|res8[7];
              res->pm2_5_con_unit_atmosphe = (res8[8]<<8)|res8[9];
              res->con_unit_atmosphe = (res8[10]<<8)|res8[11];
              res->particle_nr_0_3_um = (res8[12]<<8)|res8[13];
              res->particle_nr_0_5_um = (res8[14]<<8)|res8[15];
              res->particle_nr_1_0_um = (res8[16]<<8)|res8[17];
              res->particle_nr_2_5_um = (res8[18]<<8)|res8[19];
              res->particle_nr_5_0_um = (res8[20]<<8)|res8[21];
              res->particle_nr_10_um = (res8[22]<<8)|res8[23];
              res->reserved = (res8[24]<<8)|res8[25];
              res->check = (res8[26]<<8)|res8[27];
              ESP_LOGV(TAG, "RES: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
              res->pm1_0_con_unit,
              res->pm2_5_con_unit,
              res->pm10_con_unit,
              res->pm1_0_con_unit_atmosphe,
              res->pm2_5_con_unit_atmosphe,
              res->con_unit_atmosphe,
              res->particle_nr_0_3_um,
              res->particle_nr_0_5_um,
              res->particle_nr_1_0_um,
              res->particle_nr_2_5_um,
              res->particle_nr_5_0_um,
              res->particle_nr_10_um,
              res->reserved,
              res->check);
              uart_flush(dev->uart_dev.port);
              return ESP_OK;
            }
          }
        break;
        default:
          ESP_LOGE(TAG, "Invalid parsing state! %u",decode_state);
          return ESP_ERR_INVALID_STATE;
        break;
      }
    }
  }
  return ESP_OK;
}

static esp_err_t read_res(pms1003_t *dev, pms1003_raw_data_t *res) {
  UART_DEV_TAKE_MUTEX(&dev->uart_dev);
  UART_DEV_CHECK(&dev->uart_dev, read_res_nolock(dev, res));
  UART_DEV_GIVE_MUTEX(&dev->uart_dev);

  return ESP_OK;
}

static esp_err_t exec_cmd(pms1003_t *dev, uint8_t cmd[], size_t delay_ticks) {
  UART_DEV_TAKE_MUTEX(&dev->uart_dev);
  UART_DEV_CHECK(&dev->uart_dev, send_cmd_nolock(dev, cmd));
  if (delay_ticks) vTaskDelay(delay_ticks);
  UART_DEV_GIVE_MUTEX(&dev->uart_dev);

  return ESP_OK;
}

static inline bool is_measuring(pms1003_t *dev) {
    // not running if measurement is not started
    if (!dev->meas_started)
      return false;

    // not running if time elapsed is greater than duration
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;
    return (elapsed < dev->sen.conf.delay_start_get_us);
}

static inline void print_raw_values(pms1003_t *dev) {
  ESP_LOGD(TAG, "esp_timestamp: %llu",dev->sen.esp_timestamp);
  ESP_LOGD(TAG, "PM1.0: %u",dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].m_raw);
  ESP_LOGD(TAG, "PM2.5: %u",dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].m_raw);
  ESP_LOGD(TAG, "PM2.5: %u",dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].m_raw);
  ESP_LOGD(TAG, "PM2.5: %u",dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].m_raw);
  ESP_LOGD(TAG, "PM2.5: %u",dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].m_raw);
  ESP_LOGD(TAG, "ATMOS: %u",dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].m_raw);
  ESP_LOGD(TAG, "PM0.3 nr: %u",dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].m_raw);
  ESP_LOGD(TAG, "PM0.5 nr: %u",dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].m_raw);
  ESP_LOGD(TAG, "PM1.0 nr: %u",dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].m_raw);
  ESP_LOGD(TAG, "PM2.5 nr: %u",dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].m_raw);
  ESP_LOGD(TAG, "PM5.0 nr: %u",dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].m_raw);
  ESP_LOGD(TAG, "PM10 nr: %u",dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].m_raw);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t pms1003_init_desc(pms1003_t *dev, uart_port_t port, gpio_num_t tx_gpio, \
                            gpio_num_t rx_gpio, gpio_num_t rts_io_num, gpio_num_t cts_io_num, \
                            int tx_buffer_size, int rx_buffer_size, int queue_size, \
                            int intr_alloc_flags, int baud_rate, uart_word_length_t data_bits, \
                            uart_parity_t parity, uart_stop_bits_t stop_bits, \
                            uart_hw_flowcontrol_t flow_ctrl, uint8_t rx_flow_ctrl_thresh, \
                            uart_sclk_t source_clk, gpio_num_t rst_pin, gpio_num_t set_pin, uint16_t sen_id) {
    CHECK_ARG(dev);
    dev->conf.delay_after_awake_ms = (uint32_t) CONFIG_PMS1003_AFTER_AWAKE_DELAY_MS;
    dev->conf.rst_pin = rst_pin;
    dev->conf.set_pin = set_pin;
    dev->status.sleep_mode = PMS1003_SLEEP_MODE_MAX;
    // dev->sen.status.sleeping = false;
    dev->status.mode = PMS1003_DATA_MODE_ACTIVE;
    dev->uart_dev.port = port;
    dev->uart_dev.tx_io_num = tx_gpio;
    dev->uart_dev.rx_io_num = rx_gpio;
    // dev->uart_dev.rts_io_num = rts_io_num;
    // dev->uart_dev.cts_io_num = cts_io_num;
    dev->uart_dev.tx_buffer_size = tx_buffer_size;
    dev->uart_dev.rx_buffer_size = rx_buffer_size;
    dev->uart_dev.queue_size = queue_size;
    dev->uart_dev.queue = NULL;
    dev->uart_dev.intr_alloc_flags = intr_alloc_flags;
    dev->uart_dev.cfg.baud_rate = baud_rate;
    dev->uart_dev.cfg.data_bits = data_bits;
    dev->uart_dev.cfg.parity = parity;
    dev->uart_dev.cfg.stop_bits = stop_bits;
    dev->uart_dev.cfg.flow_ctrl = flow_ctrl;
    dev->uart_dev.cfg.source_clk = source_clk;

    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,12);
    strncpy(dev->sen.info.name, "PMS1003\0", 8);
    // dev->sen.conf.delay_after_awake_us = (uint32_t) (CONFIG_PMS1003_AFTER_AWAKE_DELAY_MS*1000);
    dev->sen.conf.delay_after_awake_us = 35000000;
    dev->sen.conf.time_to_adjust_us = 0;
    dev->sen.conf.delay_start_get_us = 4420420;
    dev->sen.info.lib_id = SEN_PMS1003_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.info.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.conf.min_period_us = 30000000;
    dev->sen.info.out_nr = 12;
    dev->sen.info.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.conf.period_ms=CONFIG_PMS1003_DEFAULT_PERIOD_MS;
    dev->sen.dev=dev;
    dev->sen.reset=pms1003_iot_sen_reset;
    dev->sen.reinit=pms1003_iot_sen_reinit;
    dev->sen.start_measurement=pms1003_iot_sen_start_measurement;
    dev->sen.get_data=pms1003_iot_sen_get_data;
    dev->sen.awake=pms1003_iot_sen_sleep_mode_awake;
    dev->sen.sleep=pms1003_iot_sen_sleep_mode_sleep;

    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;
    dev->sen.status.delay_m_us = 0;

    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].out_id=PMS1003_OUT_PM1_0_CON_UNIT_ID;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].out_type = SEN_TYPE_PM1_0_CON_UNIT;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].out_id=PMS1003_OUT_PM2_5_CON_UNIT_ID;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].out_type = SEN_TYPE_PM2_5_CON_UNIT;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_id=PMS1003_OUT_PM10_CON_UNIT_ID;
    dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_type = SEN_TYPE_PM10_CON_UNIT;
    dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].out_id=PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].out_type = SEN_TYPE_PM1_0_CON_UNIT_ATMOSPHE;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].out_id=PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].out_type = SEN_TYPE_PM2_5_CON_UNIT_ATMOSPHE;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].out_id=PMS1003_OUT_CON_UNIT_ATMOSPHE_ID;
    dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].out_type = SEN_TYPE_CON_UNIT_ATMOSPHE;
    dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_0_3_UM_ID;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].out_type = SEN_TYPE_PARTICLE_NR_0_3_UM;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_0_5_UM_ID;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].out_type = SEN_TYPE_PARTICLE_NR_0_5_UM;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_1_0_UM_ID;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].out_type = SEN_TYPE_PARTICLE_NR_1_0_UM;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_2_5_UM_ID;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].out_type = SEN_TYPE_PARTICLE_NR_2_5_UM;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_5_0_UM_ID;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].out_type = SEN_TYPE_PARTICLE_NR_5_0_UM;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].conf.srate = 0;

    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_10_UM_ID;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].out_type = SEN_TYPE_PARTICLE_NR_10_UM;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].m_raw=0;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].dust=0.0;
    // dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].conf.srate = 0;
    dev->sen.conf.srate = 0;

    return uart_dev_create_mutex(&dev->uart_dev);
}

esp_err_t pms1003_free_desc(pms1003_t *dev) {
    CHECK_ARG(dev);

    return uart_dev_delete_mutex(&dev->uart_dev);
}

esp_err_t pms1003_init(pms1003_t *dev) {
  CHECK_ARG(dev);
  gpio_config_t io_conf;
  size_t available;
  ESP_LOGI(TAG, "PMS1003 initializing...");
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = ((1ULL<<dev->conf.rst_pin) | (1ULL<<dev->conf.set_pin));
  io_conf.mode = GPIO_MODE_OUTPUT;
  // io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  CHECK(gpio_config(&io_conf));
  CHECK(gpio_set_level(dev->conf.rst_pin, 0));
  CHECK(gpio_set_level(dev->conf.set_pin, 0));
  uart_driver_delete(dev->uart_dev.port);
  CHECK(uart_param_config(dev->uart_dev.port, &dev->uart_dev.cfg));
  CHECK(uart_set_pin(dev->uart_dev.port, dev->uart_dev.tx_io_num, dev->uart_dev.rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  CHECK(uart_driver_install(dev->uart_dev.port,dev->uart_dev.rx_buffer_size, dev->uart_dev.tx_buffer_size, 0, NULL, dev->uart_dev.intr_alloc_flags));
  CHECK(pms1003_reset(dev));
  CHECK(pms1003_set_sleep_mode(dev, PMS1003_SLEEP_MODE_AWAKE));
  // CHECK(uart_param_config(dev->uart_dev.port, &dev->uart_dev.cfg));
  // CHECK(uart_set_pin(dev->uart_dev.port, dev->uart_dev.tx_io_num, dev->uart_dev.rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  // CHECK(uart_driver_install(dev->uart_dev.port, 256, 256, 0, NULL, dev->uart_dev.intr_alloc_flags));
  // CHECK(uart_driver_install(dev->uart_dev.port, dev->uart_dev.rx_buffer_size, dev->uart_dev.tx_buffer_size, dev->uart_dev.queue_size, &dev->uart_dev.queue, dev->uart_dev.intr_alloc_flags));
  // vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_after_awake_us/1000));
  vTaskDelay(pdMS_TO_TICKS(100));

  /* Set pattern interrupt, used to detect the end of a line */
  // uart_enable_pattern_det_baud_intr(dev->uart_dev.port, START_BYTE_1, 1, 9, 0, 0);
  /* Set pattern queue size */
  // uart_pattern_queue_reset(dev->uart_dev.port, dev->uart_dev.event_queue_size);
  // CHECK(uart_disable_rx_intr(dev->uart_dev.port));
  // CHECK(uart_disable_tx_intr(dev->uart_dev.port));
  uint64_t init_start=esp_timer_get_time();
  while(1){
    UART_DEV_CHECK(&dev->uart_dev, uart_get_buffered_data_len(dev->uart_dev.port, &available));
    if((available>=PMS1003_DATA_FRAME_LENGTH+4) || ((esp_timer_get_time()-init_start)>dev->sen.conf.delay_start_get_us)) break;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  // vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  ESP_LOGD(TAG, "Available Rx buffer bytes: %d", available);
  ESP_LOGD(TAG, "time untill get data: %llu", (esp_timer_get_time()-init_start));
  // CHECK(pms1003_set_sleep_mode(dev, PMS1003_SLEEP_MODE_SLEEP));
  if(available>=PMS1003_DATA_FRAME_LENGTH+4){
    dev->sen.status.status_code=SEN_STATUS_OK;
    ESP_LOGI(TAG, "Sensor initialized!");
    return ESP_OK;
  }
  return ESP_FAIL;
}

esp_err_t pms1003_reset(pms1003_t *dev) {
  CHECK_ARG(dev);
  // dev->meas_start_time = 0;
  // dev->meas_started = false;
  ESP_LOGI(TAG, "Sensor is restarting...");
#ifdef CONFIG_PMS1003_USE_HW_CTRL
  CHECK(gpio_set_level(dev->conf.rst_pin, 0));
  vTaskDelay(420);
  CHECK(gpio_set_level(dev->conf.rst_pin, 1));
#else
  ESP_LOGE(TAG, "Can't reset device without hardware control enabled on menuconfig.");
#endif
  uart_flush(dev->uart_dev.port);
  return ESP_OK;
}

esp_err_t pms1003_measure(pms1003_t *dev, pms1003_raw_data_t *raw) {
  // esp_err_t ret;
  ESP_LOGD(TAG, "pms1003 sen measurement");
  // CHECK_ARG(dev && (pm1_0_con_unit || pm2_5_con_unit || pm10_con_unit || pm1_0_con_unit_atmosphe || pm2_5_con_unit_atmosphe || con_unit_atmosphe || particle_nr_0_3_um || particle_nr_0_5_um || particle_nr_1_0_um || particle_nr_2_5_um || particle_nr_5_0_um || particle_nr_10_um));
  CHECK_ARG(dev && raw);
  // CHECK_ARG(dev);
  CHECK(pms1003_start_measurement(dev));
  // uint8_t cmd[7];
  // CHECK(get_meas_cmd(cmd));
  // CHECK(exec_cmd(dev,cmd, 0));
  vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  while(is_measuring(dev));
  CHECK(pms1003_get_raw_data(dev,raw));
  // if(dev->conf.delay_after_awake_ms)
  //   pms1003_set_sleep_mode(dev, PMS1003_SLEEP_MODE_SLEEP);
  return pms1003_compute_values(dev, &raw);
}

esp_err_t pms1003_start_measurement(pms1003_t *dev) {
  CHECK_ARG(dev);
  esp_err_t ret;
  uint8_t cmd[7];
  ESP_LOGD(TAG, "Start measurement");

  if (is_measuring(dev)) {
    ESP_LOGE(TAG, "Measurement is still running");
    return ESP_ERR_INVALID_STATE;
  }

  if(dev->status.sleep_mode == PMS1003_SLEEP_MODE_SLEEP) {
    CHECK(pms1003_set_sleep_mode(dev, PMS1003_SLEEP_MODE_AWAKE));
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_after_awake_us/1000));
  }
  if(dev->status.mode==PMS1003_DATA_MODE_PASSIVE){
    CHECK(get_meas_cmd(cmd));
    CHECK(exec_cmd(dev,cmd, 0));
  } else if(dev->status.mode == PMS1003_DATA_MODE_ACTIVE){
    ESP_LOGD(TAG, "Cleaning RX buffer...");
    uart_flush(dev->uart_dev.port); //Clean all messages scince sensor awake
  }
  dev->meas_start_time = esp_timer_get_time();
  dev->meas_started = true;

  return ESP_OK;
}

esp_err_t pms1003_set_data_mode(pms1003_t *dev, pms1003_mode_type_t data_mode) {
  uint8_t cmd[7];
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "Setting data mode: %u",data_mode);
  CHECK(get_set_data_mode_cmd(cmd, data_mode));
  CHECK(exec_cmd(dev,cmd,0));
  dev->status.mode = data_mode;
  return ESP_OK;
}

esp_err_t pms1003_set_sleep_mode(pms1003_t *dev, pms1003_sleep_type_t sleep_mode) {
  uint8_t cmd[7];
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "pms1003_set_sleep_mode");
  ESP_LOGD(TAG, "Initial sleep mode: %u", dev->status.sleep_mode);
  if(dev->status.sleep_mode == sleep_mode) return ESP_OK;
  if((sleep_mode!=PMS1003_SLEEP_MODE_SLEEP) && (sleep_mode!=PMS1003_SLEEP_MODE_AWAKE)){
    ESP_LOGE(TAG, "Ivalid sleep mode!");
    return ESP_ERR_INVALID_STATE;
  }
#ifdef CONFIG_PMS1003_USE_HW_CTRL
  if(sleep_mode==PMS1003_SLEEP_MODE_SLEEP) {
    ESP_LOGD(TAG, "Sensor going to sleep");
    CHECK(gpio_set_level(dev->conf.set_pin, 0));
  } else {
    ESP_LOGD(TAG, "Sensor awaking");
    CHECK(gpio_set_level(dev->conf.set_pin, 1));
  }
#else
  CHECK(get_set_sleep_mode_cmd(cmd, sleep_mode));
  CHECK(exec_cmd(dev,cmd,0));
#endif
  dev->status.sleep_mode = sleep_mode;
  ESP_LOGD(TAG, "Current sleep mode: %u", dev->status.sleep_mode);
  return ESP_OK;
}

esp_err_t pms1003_toggle_sleep_mode(pms1003_t *dev) {
  CHECK_ARG(dev);
  ESP_LOGD(TAG, "pms1003_toggle_sleep_mode");
  if(dev->status.sleep_mode==PMS1003_SLEEP_MODE_SLEEP) {
    CHECK(pms1003_set_sleep_mode(dev, PMS1003_SLEEP_MODE_AWAKE));
  }
  else {
    CHECK(pms1003_set_sleep_mode(dev, PMS1003_SLEEP_MODE_SLEEP));
  }

  return ESP_OK;
}

size_t pms1003_get_measurement_duration(pms1003_t *dev) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "pms1003_toggle_sleep_mode");
    size_t res = pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000);
    return res == 0 ? 1 : res;
}

esp_err_t pms1003_get_raw_data(pms1003_t *dev, pms1003_raw_data_t *raw) {
    CHECK_ARG(dev);
    // int64_t timestamp;

    ESP_LOGD(TAG, "pms1003_get_raw_data");
    // if (is_measuring(dev)) {
    //   ESP_LOGE(TAG, "Measurement is still running");
    //   return ESP_ERR_INVALID_STATE;
    // }

    CHECK(read_res(dev, raw));
    // timestamp=esp_timer_get_time();
    dev->meas_started = false;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].m_raw = raw->pm1_0_con_unit;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].m_raw = raw->pm2_5_con_unit;
    dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].m_raw = raw->pm10_con_unit;
    dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].m_raw = raw->pm1_0_con_unit_atmosphe;
    dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].m_raw = raw->pm2_5_con_unit_atmosphe;
    dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].m_raw = raw->con_unit_atmosphe;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].m_raw = raw->particle_nr_0_3_um;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].m_raw = raw->particle_nr_0_5_um;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].m_raw = raw->particle_nr_1_0_um;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].m_raw = raw->particle_nr_2_5_um;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].m_raw = raw->particle_nr_5_0_um;
    dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].m_raw = raw->particle_nr_10_um;
    dev->sen.esp_timestamp=esp_timer_get_time();
    // print_raw_values(dev);
    return ESP_OK;
}

esp_err_t pms1003_compute_values(pms1003_t *dev, pms1003_raw_data_t *raw) {
  dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].dust = (float)(raw->pm1_0_con_unit);
  dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].dust = (float)(raw->pm2_5_con_unit);
  dev->sen.outs[PMS1003_OUT_PM10_CON_UNIT_ID].dust = (float)(raw->pm10_con_unit);
  dev->sen.outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].dust = (float)(raw->pm1_0_con_unit_atmosphe);
  dev->sen.outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].dust = (float)(raw->pm2_5_con_unit_atmosphe);
  dev->sen.outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].dust = (float)(raw->con_unit_atmosphe);
  dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].dust = (float)(raw->particle_nr_0_3_um);
  dev->sen.outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].dust = (float)(raw->particle_nr_0_5_um);
  dev->sen.outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].dust = (float)(raw->particle_nr_1_0_um);
  dev->sen.outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].dust = (float)(raw->particle_nr_2_5_um);
  dev->sen.outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].dust = (float)(raw->particle_nr_5_0_um);
  dev->sen.outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].dust = (float)(raw->particle_nr_10_um);
  return ESP_OK;
}

esp_err_t pms1003_get_results(pms1003_t *dev) {
  CHECK_ARG(dev);
  pms1003_raw_data_t raw;
  ESP_LOGD(TAG, "pms1003_get_results");
  CHECK(pms1003_get_raw_data(dev, &raw));
  return pms1003_compute_values(dev, &raw);
}

esp_err_t pms1003_iot_sen_start_measurement(void *dev) {
  return pms1003_start_measurement((pms1003_t *)dev);
}

esp_err_t pms1003_iot_sen_get_data(void *dev) {
  esp_err_t ret;
  pms1003_raw_data_t raw;
  // CHECK(pms1003_get_raw_data(dev,&raw));
  // return pms1003_compute_values(dev, &raw);
  return pms1003_get_results((pms1003_t *)dev);
  // return pms1003_measure((pms1003_t *)dev, &raw);
  // return ESP_OK;
  // return pms1003_measure((pms1003_t *)dev, &raw);
}

esp_err_t pms1003_iot_sen_toggle_sleep_mode(void *dev) {
  return pms1003_toggle_sleep_mode((pms1003_t *)dev);
}

esp_err_t pms1003_iot_sen_sleep_mode_awake(void *dev) {
  return pms1003_set_sleep_mode((pms1003_t *)dev, PMS1003_SLEEP_MODE_AWAKE);
}

esp_err_t pms1003_iot_sen_sleep_mode_sleep(void *dev) {
  return pms1003_set_sleep_mode((pms1003_t *)dev, PMS1003_SLEEP_MODE_SLEEP);
}

esp_err_t pms1003_iot_sen_reset(void *dev) {
  return pms1003_reset((pms1003_t *)dev);
}

esp_err_t pms1003_iot_sen_reinit(void *dev) {
  return pms1003_init((pms1003_t *)dev);
}
