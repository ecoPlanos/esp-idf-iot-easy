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
 * @file sfa30.c
 *
 * ESP-IDF driver for Sensirion SFA30 digital temperature and humidity sensor
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
#include "sfa30.h"

#define I2C_FREQ_HZ 100000 // 100KHz

static const char *TAG = "sfa30";

// #define CMD_START             SFA30_I2C_ADDRESS
#define CMD_RESET                       0xD304
#define CMD_SERIAL                      0xD060
#define CMD_MEAS_PERIODIC_START         0x0006
#define CMD_MEAS_PERIODIC_STOP          0x0104
#define CMD_MEAS_PERIODIC_GET_DATA      0x0327

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

// #define G_POLYNOM 0x31
#define G_POLYNOM 0x31

static uint8_t crc8(uint8_t data[], size_t len) {
  uint8_t crc = 0xff;
  size_t i, j;

  for (i = 0; i < len; i++) {
    ESP_LOGD(TAG,"data[%u]: %02x",i,data[i]);
    crc ^= data[i];
    for (j = 0; j < 8; j++)
      crc = (crc & 0x80) ? ((crc << 1) ^ G_POLYNOM) : (crc << 1);
  }
  ESP_LOGD(TAG,"CRC: %02x",crc);
  return crc;
}

static inline size_t get_duration_ms(sfa30_t *dev) {
  return 5;
}

static inline esp_err_t send_cmd_nolock(sfa30_t *dev, uint16_t cmd) {
  esp_err_t err=ESP_OK;
  uint8_t cmd_8[2];
  cmd_8[0]=((cmd>>8)&0xFF);
  cmd_8[1]=((cmd)&0xFF);
  ESP_LOGD(TAG, "Sending cmd %04x...", cmd);
  err=i2c_dev_write(&dev->i2c_dev, NULL, 0, cmd_8, 2);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_WRITE;
    dev->sen.status.fail_reg=cmd;
  }
  return err;
}

static inline esp_err_t read_res_nolock(sfa30_t *dev, sfa30_raw_data_t res, uint8_t byte_nr) {
  esp_err_t err=ESP_OK;
  uint8_t i;
  err=i2c_dev_read(&dev->i2c_dev, NULL, 0, res, byte_nr);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_READ;
    return err;
  }
  ESP_LOGD(TAG, "Got response %02x %02x %02x %02x %02x %02x %02x %02x %02x",
  res[0], res[1], res[2], res[3], res[4], res[5], res[6], res[7], res[9]);
  for(i=0;i<byte_nr;i+=3) {
    if(res[i+2] != crc8(res+i, 2)) {
      ESP_LOGE(TAG, "Invalid CRC");
      return ESP_ERR_INVALID_CRC;
    }
  }
  return ESP_OK;
}

static esp_err_t send_cmd(sfa30_t *dev, uint16_t cmd) {
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  return ESP_OK;
}

static esp_err_t read_res(sfa30_t *dev, sfa30_raw_data_t res, uint8_t byte_nr) {
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, read_res_nolock(dev, res, byte_nr));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  return ESP_OK;
}

static esp_err_t exec_cmd(sfa30_t *dev, uint16_t cmd, size_t delay_ticks, sfa30_raw_data_t res, uint8_t byte_nr) {
  uint8_t i;
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
  if (delay_ticks)
      vTaskDelay(delay_ticks);
  I2C_DEV_CHECK(&dev->i2c_dev, read_res_nolock(dev, res, byte_nr));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  for(i=0;i<byte_nr;i++) {
    ESP_LOGD(TAG, "byte[%u]:, %02x",i,res[i]);
  }
  // ESP_LOGD(TAG,"exec_cmd_res: %u, %u, %u, %u, %u, %u", res[0], res[1], res[2], res[3], res[4], res[5]);

  return ESP_OK;
}

static inline bool is_measuring(sfa30_t *dev) {
    // not running if measurement is not started
    if (!dev->meas_started)
      return false;

    // not running if time elapsed is greater than duration
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;
    return elapsed < get_duration_ms(dev) * 1000;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sfa30_init_desc(sfa30_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id) {
  CHECK_ARG(dev);
  dev->measurement_running = false;
  dev->i2c_dev.port = port;
  dev->i2c_dev.addr = SFA30_I2C_ADDRESS;
  dev->i2c_dev.cfg.sda_io_num = sda_gpio;
  dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
  dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
  memset(&dev->sen, 0, sizeof(sensor_t));
  sensor_init(&dev->sen,3);
  strncpy(dev->sen.info.name, "SFA30\0", 6);
  dev->sen.info.lib_id = SEN_SFA30_LIB_ID;
  dev->sen.info.sen_id = sen_id;
  dev->sen.info.version = 1;
  dev->sen.info.com_type = SEN_COM_TYPE_DIGITAL_COM;
  dev->sen.conf.min_period_us = 250000;
  dev->sen.conf.delay_start_get_us = 20000;
  dev->sen.info.out_nr = 3; //hcho, RH, temperature
  dev->sen.conf.trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
  dev->sen.conf.addr = SFA30_I2C_ADDRESS;
  dev->sen.conf.period_ms = CONFIG_SFA30_DEFAULT_PERIOD_MS;
  dev->sen.conf.delay_after_awake_us=100000;
  dev->sen.conf.time_to_adjust_us=0;
  dev->sen.dev=dev;
  dev->sen.reset=sfa30_iot_sen_reset;
  dev->sen.reinit=sfa30_iot_sen_reinit;
  dev->sen.start_measurement=sfa30_iot_sen_start_measurement;
  dev->sen.get_data=sfa30_iot_sen_get_data;
  dev->sen.awake=sfa30_iot_sen_sleep_mode_awake;
  dev->sen.sleep=sfa30_iot_sen_sleep_mode_sleep;

  dev->sen.status.fail_cnt = 0;
  dev->sen.status.fail_time = 0;

  dev->sen.outs[SFA30_OUT_HCHO_ID].out_id=SFA30_OUT_HCHO_ID;
  dev->sen.outs[SFA30_OUT_HCHO_ID].out_val_type=SEN_OUT_VAL_TYPE_INT16;
  dev->sen.outs[SFA30_OUT_HCHO_ID].m_raw=0;
  dev->sen.outs[SFA30_OUT_HCHO_ID].processed=0.0;
  // dev->sen.outs[SFA30_OUT_HCHO_ID].conf.srate=0;

  dev->sen.outs[SFA30_OUT_RH_ID].out_id=SFA30_OUT_RH_ID;
  dev->sen.outs[SFA30_OUT_RH_ID].out_val_type=SEN_OUT_VAL_TYPE_INT16;
  dev->sen.outs[SFA30_OUT_RH_ID].m_raw=0;
  dev->sen.outs[SFA30_OUT_RH_ID].processed=0.0;
  // dev->sen.outs[SFA30_OUT_RH_ID].conf.srate=0;

  dev->sen.outs[SFA30_OUT_TEMP_ID].out_id=SFA30_OUT_TEMP_ID;
  dev->sen.outs[SFA30_OUT_TEMP_ID].out_val_type=SEN_OUT_VAL_TYPE_INT16;
  dev->sen.outs[SFA30_OUT_TEMP_ID].m_raw=0;
  dev->sen.outs[SFA30_OUT_TEMP_ID].processed=0.0;
  // dev->sen.outs[SFA30_OUT_TEMP_ID].conf.srate=0;
  dev->sen.conf.srate=0;

  return i2c_dev_create_mutex(&dev->i2c_dev);
  // return ESP_OK;
}

esp_err_t sfa30_free_desc(sfa30_t *dev) {
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sfa30_init(sfa30_t *dev) {
  esp_err_t ret;
  uint8_t device_marking[SFA30_MARKING_DATA_SIZE];
  CHECK_ARG(dev);
  float hcho, humidity, temperature;

  dev->sen.conf.delay_start_get_us=get_duration_ms(dev)*1000;

  CHECK(sfa30_reset(dev));
  vTaskDelay(pdMS_TO_TICKS(100));

  CHECK(sfa30_get_device_marking(dev, device_marking));
  vTaskDelay(pdMS_TO_TICKS(10));
  CHECK(sfa30_measure(dev, &hcho, &humidity, &temperature));
  //TODO: find stratagy to check if it is connected
  dev->sen.status.status_code = SEN_STATUS_OK;
  return ESP_OK;
}

esp_err_t sfa30_reset(sfa30_t *dev) {
    dev->meas_start_time = 0;
    dev->meas_started = false;
    dev->measurement_running=false;

    CHECK(send_cmd(dev, CMD_RESET));

    return ESP_OK;
}

esp_err_t sfa30_get_device_marking(sfa30_t *dev, uint8_t device_marking[]) {
  esp_err_t error;

  CHECK(send_cmd(dev,CMD_SERIAL));
  vTaskDelay(pdMS_TO_TICKS(2));
  I2C_DEV_CHECK(&dev->i2c_dev, read_res(dev, device_marking, SFA30_MARKING_DATA_SIZE));
  return ESP_OK;
}
esp_err_t sfa30_measure(sfa30_t *dev, float *hcho, float *humidity, float *temperature) {
  CHECK_ARG(dev && (hcho || temperature || humidity));

  sfa30_raw_data_t raw;
  // gettimeofday(&tv, NULL);
  if(!dev->measurement_running) {
    vTaskDelay(100);
    CHECK(sfa30_start_measurement(dev));
  }
  vTaskDelay(pdMS_TO_TICKS(100));
  sfa30_get_data(dev);
  vTaskDelay(pdMS_TO_TICKS(5));
  // vTaskDelay(sfa30_get_measurement_duration(dev)+105);

  I2C_DEV_CHECK(&dev->i2c_dev, read_res(dev, raw, SFA30_RAW_DATA_SIZE));
  // CHECK(read_res(dev, raw, SFA30_RAW_DATA_SIZE));

  dev->sen.outs[SFA30_OUT_HCHO_ID].m_raw = (raw[0]<<8) | raw[1];
  dev->sen.outs[SFA30_OUT_RH_ID].m_raw = (raw[3]<<8) | raw[4];
  dev->sen.outs[SFA30_OUT_TEMP_ID].m_raw = (raw[6]<<8) | raw[7];
  dev->sen.outs[SFA30_OUT_HCHO_ID].processed = *hcho;
  dev->sen.outs[SFA30_OUT_RH_ID].processed = *humidity;
  dev->sen.outs[SFA30_OUT_TEMP_ID].processed = *temperature;
  CHECK(sfa30_compute_values(dev, raw, hcho, humidity, temperature));
  dev->sen.esp_timestamp = esp_timer_get_time();
  ESP_LOGD(TAG, "HCHO raw: %d",dev->sen.outs[SFA30_OUT_HCHO_ID].m_raw);
  ESP_LOGD(TAG, "RH raw: %d",dev->sen.outs[SFA30_OUT_RH_ID].m_raw);
  ESP_LOGD(TAG, "Temp raw: %d",dev->sen.outs[SFA30_OUT_TEMP_ID].m_raw);
  ESP_LOGD(TAG, "HCHO: %f",dev->sen.outs[SFA30_OUT_HCHO_ID].processed);
  ESP_LOGD(TAG, "RH: %f",dev->sen.outs[SFA30_OUT_RH_ID].processed);
  ESP_LOGD(TAG, "Temp: %f",dev->sen.outs[SFA30_OUT_TEMP_ID].processed);
  return ESP_OK;
}

esp_err_t sfa30_start_measurement(sfa30_t *dev) {
    CHECK_ARG(dev);
    if(!dev->measurement_running) {
      CHECK(send_cmd(dev, CMD_MEAS_PERIODIC_START));
      dev->measurement_running=true;
    }
    ESP_LOGD(TAG, "Measurement started!");

    return ESP_OK;
}

esp_err_t sfa30_get_data(sfa30_t *dev) {
    CHECK_ARG(dev);
    if(is_measuring(dev)){
      ESP_LOGW(TAG, "Sensor measurement request was already sent.");
      return ESP_ERR_INVALID_STATE;
    }
    CHECK(send_cmd(dev, CMD_MEAS_PERIODIC_GET_DATA));
    dev->measurement_running=true;
    ESP_LOGD(TAG, "Retrieving data!");
    dev->meas_start_time = esp_timer_get_time();
    // dev->sen.esp_timestamp = esp_timer_get_time();
    dev->meas_started = true;

    return ESP_OK;
}

esp_err_t sfa30_stop_measurement(sfa30_t *dev) {
  CHECK_ARG(dev);
  if(dev->measurement_running){
    CHECK(send_cmd(dev, CMD_MEAS_PERIODIC_STOP));
    dev->measurement_running=false;
    ESP_LOGD(TAG, "Measurement stoped!");
  }

  return ESP_OK;
}

size_t sfa30_get_measurement_duration(sfa30_t *dev) {
    if (!dev) return 0;

    size_t res = pdMS_TO_TICKS(get_duration_ms(dev));
    ESP_LOGD(TAG,"Measurement duration ticks: %u", res);
    return res == 0 ? 1 : res;
}

esp_err_t sfa30_get_raw_data(sfa30_t *dev, sfa30_raw_data_t raw) {
    CHECK_ARG(dev);
    esp_err_t ret;
    struct timeval tv;
    CHECK(sfa30_start_measurement(dev));
    vTaskDelay(sfa30_get_measurement_duration(dev));
    ret = read_res(dev, raw, SFA30_RAW_DATA_SIZE);
    if(ret==ESP_OK) {
      // gettimeofday(&tv, NULL);
      dev->sen.esp_timestamp = esp_timer_get_time();
      dev->sen.outs[SFA30_OUT_HCHO_ID].m_raw=((raw[0] << 8) | raw[1]);
      dev->sen.outs[SFA30_OUT_RH_ID].m_raw=((raw[3] << 8) | raw[4]);
      dev->sen.outs[SFA30_OUT_TEMP_ID].m_raw=((raw[6] << 8) | raw[7]);
      ESP_LOGD(TAG, "raw[0]: %u,raw[1]: %u,raw[2]: %u,raw[3]: %u,raw[4]: %u,raw[5]: %u,raw[6]: %u,raw[7]: %u,raw[8]: %u",raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],raw[8]);
      ESP_LOGD(TAG, "HCHO raw: %d",dev->sen.outs[SFA30_OUT_HCHO_ID].m_raw);
      ESP_LOGD(TAG, "RH raw: %d",dev->sen.outs[SFA30_OUT_RH_ID].m_raw);
      ESP_LOGD(TAG, "Temp raw: %d",dev->sen.outs[SFA30_OUT_TEMP_ID].m_raw);
      // dev->sen.outs[SFA30_OUT_TEMP_ID].m_raw=((raw[0] << 16) | (raw[1] << 8) | raw[2]);
      // dev->sen.outs[SFA30_OUT_RH_ID].m_raw=((raw[3] << 16) | (raw[4] << 8) | raw[5]);
    }
    return ret;
}

esp_err_t sfa30_compute_values(sfa30_t *dev, sfa30_raw_data_t raw_data, float *hcho, float *humidity, float *temperature) {
    CHECK_ARG(raw_data && (hcho || temperature || humidity));

    if (hcho)
      *hcho = (((float)((int16_t)((raw_data[0] << 8) | raw_data[1])))/5.0);
    if (humidity)
      *humidity = (((float)((int16_t)((raw_data[3] << 8) | raw_data[4])))/100.0);
    if (temperature)
      *temperature = (((float)((int16_t)((raw_data[6] << 8) | raw_data[7])))/200.0);

    dev->sen.outs[SFA30_OUT_HCHO_ID].processed=*hcho;
    dev->sen.outs[SFA30_OUT_RH_ID].processed=*humidity;
    dev->sen.outs[SFA30_OUT_TEMP_ID].processed=*temperature;
    ESP_LOGD(TAG, "HCHO: %f",dev->sen.outs[SFA30_OUT_HCHO_ID].processed);
    ESP_LOGD(TAG, "RH: %f",dev->sen.outs[SFA30_OUT_RH_ID].processed);
    ESP_LOGD(TAG, "Temp: %f",dev->sen.outs[SFA30_OUT_TEMP_ID].processed);

    return ESP_OK;
}

esp_err_t sfa30_get_results(sfa30_t *dev, float *hcho, float *humidity, float *temperature) {
    sfa30_raw_data_t raw;
    CHECK(sfa30_get_raw_data(dev, raw));

    return sfa30_compute_values(dev, raw, hcho, humidity, temperature);
}

esp_err_t sfa30_iot_sen_start_measurement(void *dev) {
  return sfa30_get_data((sfa30_t*) dev);
  // return ESP_OK;
}

esp_err_t sfa30_iot_sen_get_data(void *dev) {
  float hcho, humidity, temperature;
  sfa30_t *dev_ = (sfa30_t *)dev;
  sfa30_raw_data_t raw;

  CHECK(read_res(dev_, raw, SFA30_RAW_DATA_SIZE));
  // gettimeofday(&tv, NULL);
  // dev_->sen.esp_timestamp = esp_timer_get_time();
  dev_->sen.outs[SFA30_OUT_HCHO_ID].m_raw=((raw[0] << 8) | raw[1]);
  dev_->sen.outs[SFA30_OUT_RH_ID].m_raw=((raw[3] << 8) | raw[4]);
  dev_->sen.outs[SFA30_OUT_TEMP_ID].m_raw=((raw[6] << 8) | raw[7]);
  if((dev_->sen.outs[SFA30_OUT_HCHO_ID].m_raw == 0)&&(dev_->sen.outs[SFA30_OUT_RH_ID].m_raw == 0)&&(dev_->sen.outs[SFA30_OUT_TEMP_ID].m_raw == 0)){
    ESP_LOGE(TAG, "All values are zero! Considering it an error...");
    return ESP_FAIL;
  }
  dev_->sen.esp_timestamp = esp_timer_get_time();
  ESP_LOGD(TAG, "raw[0]: %u,raw[1]: %u,raw[2]: %u,raw[3]: %u,raw[4]: %u,raw[5]: %u,raw[6]: %u,raw[7]: %u,raw[8]: %u",raw[0],raw[1],raw[2],raw[3],raw[4],raw[5],raw[6],raw[7],raw[8]);
  ESP_LOGD(TAG, "HCHO raw: %u",dev_->sen.outs[SFA30_OUT_HCHO_ID].m_raw);
  ESP_LOGD(TAG, "RH raw: %u",dev_->sen.outs[SFA30_OUT_RH_ID].m_raw);
  ESP_LOGD(TAG, "Temp raw: %u",dev_->sen.outs[SFA30_OUT_TEMP_ID].m_raw);
  CHECK(sfa30_compute_values(dev_, raw, &hcho, &humidity, &temperature));

  return ESP_OK;
  // return sfa30_measure(dev_, &hcho, &humidity, &temperature);
}

esp_err_t sfa30_iot_sen_sleep_mode_awake(void *dev) {
  // return sfa30_reset((sfa30_t *)dev);
  // return send_cmd(dev, CMD_RESET);
  return sfa30_start_measurement((sfa30_t*) dev);
  // return ESP_OK;
}
esp_err_t sfa30_iot_sen_sleep_mode_sleep(void *dev) {
  // return sfa30_reset((sfa30_t *)dev);
  // return sfa30_stop_measurement((sfa30_t *)dev);
  // return ESP_OK;
  // if((dev_->settings.enable_reg & TSL2591_ALS_ON) && (dev_->settings.enable_reg & TSL2591_POWER_ON))
  //   return sfa30_basic_enable(dev_);
  // else
  //   return sfa30_basic_disable(dev_);
    return ESP_OK;
}

esp_err_t sfa30_iot_sen_reset(void *dev) {
  CHECK(sfa30_reset((sfa30_t *)dev));
  // CHECK(sfa30_stop_measurement((sfa30_t *)dev));
  return ESP_OK;
}

esp_err_t sfa30_iot_sen_reinit(void *dev) {
  return sfa30_init((sfa30_t *)dev);
}
