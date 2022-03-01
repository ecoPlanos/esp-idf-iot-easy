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
 * @file sht85.c
 *
 * ESP-IDF driver for Sensirion SHT85 digital temperature and humidity sensor
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
#include "sht85.h"

#define I2C_FREQ_HZ 1000000 // 1MHz
// #define I2C_FREQ_HZ 100000 // 100KHz

static const char *TAG = "sht85";

// #define CMD_START             SHT85_I2C_ADDRESS
#define CMD_RESET             0x30A2
// #define CMD_SERIAL            0x3780
#define CMD_SERIAL                      0x3682
#define CMD_MEAS_REPEAT_HIGH            0x2400
#define CMD_MEAS_REPEAT_MED             0x240b
#define CMD_MEAS_REPEAT_LOW             0x2416
#define CMD_MEAS_PERIODIC_05MPS_R_HIGH  0x2032
#define CMD_MEAS_PERIODIC_05MPS_R_MED   0x2024
#define CMD_MEAS_PERIODIC_05MPS_R_LOW   0x202f
#define CMD_MEAS_PERIODIC_1MPS_R_HIGH   0x2130
#define CMD_MEAS_PERIODIC_1MPS_R_MED    0x2126
#define CMD_MEAS_PERIODIC_1MPS_R_LOW    0x212d
#define CMD_MEAS_PERIODIC_2MPS_R_HIGH   0x2236
#define CMD_MEAS_PERIODIC_2MPS_R_MED    0x2220
#define CMD_MEAS_PERIODIC_2MPS_R_LOW    0x222b
#define CMD_MEAS_PERIODIC_4MPS_R_HIGH   0x2334
#define CMD_MEAS_PERIODIC_4MPS_R_MED    0x2322
#define CMD_MEAS_PERIODIC_4MPS_R_LOW    0x2329
#define CMD_MEAS_PERIODIC_10MPS_R_HIGH  0x2737
#define CMD_MEAS_PERIODIC_10MPS_R_MED   0x2721
#define CMD_MEAS_PERIODIC_10MPS_R_LOW   0x272a
#define CMD_MEAS_PERIODIC_STOP          0x3093
#define CMD_MEAS_PERIODIC_GET_DATA      0xe000
#define CMD_ART                         0x2b32  //accelerated response time feature
#define CMD_HEATER_ENABLE               0x306d
#define CMD_HEATER_DISABLE              0x3066
#define CMD_STATUS_READ                 0xf32d
#define CMD_STATUS_CLEAR                0x3041


#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

// #define G_POLYNOM 0x31
#define G_POLYNOM 0x131

static uint8_t crc8(uint8_t data[], size_t len) {
  uint8_t crc = 0xff;
  size_t i, j;

  for (i = 0; i < len; i++) {
    ESP_LOGD(TAG,"data[%u]: %02x",i,data[i]);
    crc ^= data[i];
    for (j = 0; j < 8; j++)
    if(crc & 0x80) {
      crc = ((crc << 1) ^ G_POLYNOM);
    } else crc = (crc << 1);
    // crc = (crc & 0x80) ? ((crc << 1) ^ G_POLYNOM) : (crc << 1);
  }
  ESP_LOGD(TAG,"CRC: %02x",crc);
  return crc;
}

static inline size_t get_duration_ms(sht85_t *dev) {
  switch (dev->repeatability) {
    case SINGLE_MEAS_HIGH:
      return 10+2;
    case SINGLE_MEAS_MEDIUM:
      return 5+2;
    case SINGLE_MEAS_LOW:
      return 2+2;
    default:
      ESP_LOGW(TAG, "Unknown repeatability value: %d", dev->repeatability);
      return 2+2;
  }
}

static inline uint16_t get_single_shot_meas_cmd(sht85_t *dev) {
  switch (dev->repeatability) {
    case SINGLE_MEAS_HIGH:
      return CMD_MEAS_REPEAT_HIGH;
    case SINGLE_MEAS_MEDIUM:
      return CMD_MEAS_REPEAT_MED;
    case SINGLE_MEAS_LOW:
      return CMD_MEAS_REPEAT_LOW;
    default:
      ESP_LOGW(TAG, "Unknown repeatability value: %d", dev->repeatability);
      return CMD_MEAS_REPEAT_LOW;
  }
}

static inline esp_err_t send_cmd_nolock(sht85_t *dev, uint16_t cmd) {
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

static inline esp_err_t read_res_nolock(sht85_t *dev, sht85_raw_data_t res) {
  esp_err_t err=ESP_OK;
  err=i2c_dev_read(&dev->i2c_dev, NULL, 0, res, SHT85_RAW_DATA_SIZE);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_READ;
    return err;
  }

  ESP_LOGD(TAG, "Got response %02x %02x %02x %02x %02x %02x",
          res[0], res[1], res[2], res[3], res[4], res[5]);

  if (res[2] != crc8(res, 2) || res[5] != crc8(res + 3, 2)) {
    ESP_LOGE(TAG, "Invalid CRC");
    return ESP_ERR_INVALID_CRC;
  }

  return ESP_OK;
}

static esp_err_t send_cmd(sht85_t *dev, uint16_t cmd) {
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  return ESP_OK;
}

static esp_err_t read_res(sht85_t *dev, sht85_raw_data_t res) {
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, read_res_nolock(dev, res));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  return ESP_OK;
}

static esp_err_t exec_cmd(sht85_t *dev, uint16_t cmd, size_t delay_ticks, sht85_raw_data_t res) {
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, send_cmd_nolock(dev, cmd));
  if (delay_ticks)
      vTaskDelay(delay_ticks);
  I2C_DEV_CHECK(&dev->i2c_dev, read_res_nolock(dev, res));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  ESP_LOGD(TAG,"exec_cmd_res: %u, %u, %u, %u, %u, %u", res[0], res[1], res[2], res[3], res[4], res[5]);

  return ESP_OK;
}

static inline bool is_measuring(sht85_t *dev) {
    // not running if measurement is not started
    if (!dev->meas_started)
      return false;

    // not running if time elapsed is greater than duration
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;
    return elapsed < get_duration_ms(dev) * 1000;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t sht85_init_desc(sht85_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id) {
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = SHT85_I2C_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,2);
    strncpy(dev->sen.info.name, "SHT85\0", 6);
    dev->sen.info.lib_id = SEN_SHT85_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.conf.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.conf.min_period_us = 0;
    dev->sen.status.delay_start_get_us = 2000;
    dev->sen.info.out_nr = 2; //temperature, RH
    dev->sen.info.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.conf.addr = SHT85_I2C_ADDRESS;
    dev->sen.conf.period_ms = nearest_prime(CONFIG_SHT85_DEFAULT_PERIOD_MS);
    dev->sen.conf.delay_after_awake_us=2000;
    dev->sen.dev=dev;
    dev->sen.reset=sht85_iot_sen_reset;
    dev->sen.reinit=sht85_iot_sen_reinit;
    dev->sen.start_measurement=sht85_iot_sen_start_measurement;
    dev->sen.get_data=sht85_iot_sen_get_data;
    dev->sen.awake=sht85_iot_sen_sleep_mode_awake;
    dev->sen.sleep=sht85_iot_sen_sleep_mode_sleep;

    dev->sen.status.initialized = false;
    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;

    dev->sen.outs[SHT85_OUT_TEMP_ID].out_id=SHT85_OUT_TEMP_ID;
    dev->sen.outs[SHT85_OUT_TEMP_ID].out_type = SEN_TYPE_AMBIENT_TEMPERATURE;
    dev->sen.outs[SHT85_OUT_TEMP_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[SHT85_OUT_TEMP_ID].m_raw=0;
    dev->sen.outs[SHT85_OUT_TEMP_ID].temperature=0.0;
    // dev->sen.outs[SHT85_OUT_TEMP_ID].conf.srate=0;

    dev->sen.outs[SHT85_OUT_RH_ID].out_id=SHT85_OUT_RH_ID;
    dev->sen.outs[SHT85_OUT_RH_ID].out_type = SEN_TYPE_RELATIVE_HUMIDITY;
    dev->sen.outs[SHT85_OUT_RH_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.outs[SHT85_OUT_RH_ID].m_raw=0;
    dev->sen.outs[SHT85_OUT_RH_ID].relative_humidity=0.0;
    // dev->sen.outs[SHT85_OUT_RH_ID].conf.srate=0;
    dev->sen.conf.srate=0;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sht85_free_desc(sht85_t *dev) {
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sht85_init(sht85_t *dev) {
  esp_err_t ret;
  CHECK_ARG(dev);

  dev->repeatability = SINGLE_MEAS_HIGH;
  dev->heater = SHT85_HEATER_OFF;
  dev->sen.status.delay_start_get_us=get_duration_ms(dev)*1000;

  ret = sht85_reset(dev);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Error on reset.");
    return ret;
  }
  sht85_raw_data_t s;
  vTaskDelay(pdMS_TO_TICKS(2)); //Power-up delay (t_PU)
  // CHECK(exec_cmd(dev, CMD_SERIAL, pdMS_TO_TICKS(10), s));
  // ret = exec_cmd(dev, CMD_SERIAL, pdMS_TO_TICKS(2), s);
  // if(ret != ESP_OK) {
  //   ESP_LOGE(TAG, "Error sending command : %04x.", CMD_SERIAL);
  //   return ret;
  // }
  // dev->serial = ((uint32_t)s[0] << 24) | ((uint32_t)s[1] << 16) | ((uint32_t)s[3] << 8) | s[4];
  dev->sen.status.initialized = true;
  dev->sen.status.status_code = SEN_STATUS_OK;
  return ESP_OK;
}

esp_err_t sht85_reset(sht85_t *dev) {
    dev->meas_start_time = 0;
    dev->meas_started = false;

    CHECK(send_cmd(dev, CMD_RESET));

    return ESP_OK;
}

esp_err_t sht85_measure(sht85_t *dev, float *temperature, float *humidity) {
  struct timeval tv;
  CHECK_ARG(dev && (temperature || humidity));

  sht85_raw_data_t raw;
  // gettimeofday(&tv, NULL);
  CHECK(send_cmd(dev, get_single_shot_meas_cmd(dev)));
  vTaskDelay(pdMS_TO_TICKS(2));

  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_byte(&dev->i2c_dev));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  vTaskDelay(sht85_get_measurement_duration(dev));
  vTaskDelay(pdMS_TO_TICKS(2));

  I2C_DEV_CHECK(&dev->i2c_dev, read_res(dev, raw));
  dev->sen.esp_timestamp = esp_timer_get_time();

  dev->sen.outs[SHT85_OUT_TEMP_ID].m_raw = (raw[0]<<8) | raw[1];
  dev->sen.outs[SHT85_OUT_RH_ID].m_raw = (raw[3]<<8) | raw[4];
  dev->sen.outs[SHT85_OUT_TEMP_ID].temperature = *temperature;
  dev->sen.outs[SHT85_OUT_RH_ID].relative_humidity = *humidity;
  CHECK(sht85_compute_values(dev, raw, temperature, humidity));
  ESP_LOGD(TAG, "Temp raw: %u",dev->sen.outs[SHT85_OUT_TEMP_ID].m_raw);
  ESP_LOGD(TAG, "RH raw: %u",dev->sen.outs[SHT85_OUT_RH_ID].m_raw);
  ESP_LOGD(TAG, "Temp: %f",dev->sen.outs[SHT85_OUT_TEMP_ID].temperature);
  ESP_LOGD(TAG, "RH: %f",dev->sen.outs[SHT85_OUT_RH_ID].relative_humidity);
  return ESP_OK;
}

esp_err_t sht85_start_measurement(sht85_t *dev) {
    CHECK_ARG(dev);

    if (is_measuring(dev))
    {
        ESP_LOGE(TAG, "sht85_start_measurement - Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_byte(&dev->i2c_dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    ESP_LOGD(TAG, "Measurement started!");
    dev->meas_start_time = esp_timer_get_time();
    dev->meas_started = true;

    return ESP_OK;
}

size_t sht85_get_measurement_duration(sht85_t *dev) {
    if (!dev) return 0;

    size_t res = pdMS_TO_TICKS(get_duration_ms(dev));
    return res == 0 ? 1 : res;
}

esp_err_t sht85_get_raw_data(sht85_t *dev, sht85_raw_data_t raw) {
    CHECK_ARG(dev);
    esp_err_t ret;
    struct timeval tv;
    if (is_measuring(dev)) {
        ESP_LOGE(TAG, "sht85_get_raw_data - Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }
    CHECK(send_cmd(dev, get_single_shot_meas_cmd(dev)));
    vTaskDelay(pdMS_TO_TICKS(2));

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_byte(&dev->i2c_dev));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    vTaskDelay(sht85_get_measurement_duration(dev));
    vTaskDelay(pdMS_TO_TICKS(2));

    I2C_DEV_CHECK(&dev->i2c_dev, read_res(dev, raw));
    dev->sen.outs[SHT85_OUT_TEMP_ID].m_raw = (raw[0]<<8) | raw[1];
    dev->sen.outs[SHT85_OUT_RH_ID].m_raw = (raw[3]<<8) | raw[4];
    dev->meas_started = false;
    // gettimeofday(&tv, NULL);
    // dev->sen.timestamp = tv.tv_sec * 1000000LL + tv.tv_usec;
    dev->sen.esp_timestamp = esp_timer_get_time();
    ESP_LOGD(TAG, "raw[0]: %u,raw[1]: %u,raw[2]: %u,raw[3]: %u,raw[4]: %u,raw[5]: %u",raw[0],raw[1],raw[2],raw[3],raw[4],raw[5]);
    ESP_LOGD(TAG, "Temp raw: %u",dev->sen.outs[SHT85_OUT_TEMP_ID].m_raw);
    ESP_LOGD(TAG, "RH raw: %u",dev->sen.outs[SHT85_OUT_RH_ID].m_raw);

    return ESP_OK;
}

esp_err_t sht85_compute_values(sht85_t *dev, sht85_raw_data_t raw_data, float *temperature, float *humidity) {
    CHECK_ARG(raw_data && (temperature || humidity));

    if (temperature)
        *temperature = -45.0+175.0*(((float)((uint16_t)((raw_data[0] << 8) | raw_data[1])))/65535.0);
        // *temperature = ((uint16_t)((raw_data[0] << 8) | raw_data[1])) * 175.0 / 65535.0 - 45.0;

    if (humidity)
        *humidity = 100.0*(((float)((uint16_t)((raw_data[3] << 8) | raw_data[4])))/ 65535.0);
        // *humidity = ((uint16_t)((raw_data[3] << 8) | raw_data[4])) * 125.0 / 65535.0 - 6.0;

    dev->sen.outs[SHT85_OUT_TEMP_ID].temperature=*temperature;
    dev->sen.outs[SHT85_OUT_RH_ID].relative_humidity=*humidity;
    ESP_LOGD(TAG, "Temp: %f",dev->sen.outs[SHT85_OUT_TEMP_ID].temperature);
    ESP_LOGD(TAG, "RH: %f",dev->sen.outs[SHT85_OUT_RH_ID].relative_humidity);

    return ESP_OK;
}

esp_err_t sht85_get_results(sht85_t *dev, float *temperature, float *humidity) {
    sht85_raw_data_t raw;
    CHECK(sht85_get_raw_data(dev, raw));

    return sht85_compute_values(dev, raw, temperature, humidity);
}

esp_err_t sht85_iot_sen_start_measurement(void *dev) {
  // return sht85_start_measurement((sht85_t*) dev);
  return ESP_OK;
}

esp_err_t sht85_iot_sen_get_data(void *dev) {
  float temperature, humidity;
  sht85_t *dev_ = (sht85_t *)dev;
  sht85_raw_data_t raw;
  // struct timeval tv;
  // gettimeofday(&tv, NULL);
  // dev_->sen.timestamp = tv.tv_sec * 1000000LL + tv.tv_usec;
  // I2C_DEV_CHECK(&dev_->i2c_dev, read_res(dev_, raw));
  // dev_->sen.outs[SHT85_OUT_TEMP_ID].m_raw = (raw[0]<<8) | raw[1];
  // dev_->sen.outs[SHT85_OUT_RH_ID].m_raw = (raw[3]<<8) | raw[4];
  // return sht85_compute_values(dev_, raw, &temperature, &humidity);
  // dev_->sen.outs[SHT85_OUT_TEMP_ID].temperature = temperature;
  // dev_->sen.outs[SHT85_OUT_RH_ID].relative_humidity = humidity;
  return sht85_measure((sht85_t *)dev, &temperature, &humidity);
  // return sht85_get_results((sht85_t*) dev, &temperature, &humidity);
}

esp_err_t sht85_iot_sen_sleep_mode_awake(void *dev) {
  // return sht85_reset((sht85_t *)dev);
  // return send_cmd(dev, get_single_shot_meas_cmd((sht85_t *)dev));
  return ESP_OK;
}
esp_err_t sht85_iot_sen_sleep_mode_sleep(void *dev) {
  return sht85_reset((sht85_t *)dev);
  // if((dev_->settings.enable_reg & TSL2591_ALS_ON) && (dev_->settings.enable_reg & TSL2591_POWER_ON))
  //   return sht85_basic_enable(dev_);
  // else
  //   return sht85_basic_disable(dev_);
    // return ESP_OK;
}

esp_err_t sht85_iot_sen_reset(void *dev) {
  return sht85_reset((sht85_t *)dev);
}

esp_err_t sht85_iot_sen_reinit(void *dev) {
  return ESP_OK;
}
