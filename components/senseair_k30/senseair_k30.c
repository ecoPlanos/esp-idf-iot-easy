/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Julian Doerner <https://github.com/juliandoerner>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file k30.c
 *
 * ESP-IDF driver for K30 light-to-digital.
 *
 * Copyright (c) 2020 Julian Doerner <https://github.com/juliandoerner>
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <data_manager.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>
#include <string.h>
#include "senseair_k30.h"

#define I2C_FREQ_HZ 50000 // 100kHz
#define OUTPUT_NR 1

static const char *TAG = "k30";

// Registers
#define K30_RAM_CO2             0x0008
#define K30_RAM_TEMP            0x0012
#define K30_RAM_RH              0x0014
#define K30_RAM_FW_TYPE         0x0062
#define K30_RAM_REV_MAIN        0x0063
#define K30_RAM_REV_SUB         0x0064
#define K30_RAM_SEN_TYPE        0x002C
#define K30_RAM_SEN_SERIAL_NR   0x0028
#define K30_RAM_MEM_MAP_ID      0x002F

// K30 command register special functions.
typedef enum {
  K30_COMMAND_WRITE_RAM =    0x01,
  K30_COMMAND_READ_RAM =     0x02,
  K30_COMMAND_WRITE_EEPROM = 0x03,
  K30_COMMAND_READ_EEPROM =  0x04
} k30_command_t;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)


static inline uint8_t k30_checksum(uint8_t data[], size_t len) {
  uint8_t checksum = 0x00;
  size_t i;

  for (i = 0; i < len; i++) {
    // ESP_LOGD(TAG,"data[%u]: 0x%02x",i,data[i]);
    checksum += data[i];
  }
  checksum&=0xFF;
  ESP_LOGD(TAG,"checksum: 0x%02x",checksum);

  return checksum;
}

// Read/write to registers.
static inline esp_err_t write_ram(k30_t *dev, uint16_t ram_addr, uint8_t cmd, uint8_t data_size) {
  esp_err_t err=ESP_OK;
  uint8_t out_data[4];
  uint8_t resp[3];

  out_data[0]=((cmd<<4) | data_size);
  out_data[1]=((ram_addr>>8) & 0xFF);
  out_data[2]=(ram_addr & 0xFF);
  out_data[3]=k30_checksum(out_data, 3);

  ESP_LOGD(TAG, "Writing ram address: 0x%04x", ram_addr);
  ESP_LOGD(TAG, "Writing ram address_0: 0x%02x", out_data[1]);
  ESP_LOGD(TAG, "Writing ram address_1: 0x%02x", out_data[2]);

  uint8_t i=0;
  for(i=0;i<4;i++) {
    ESP_LOGD(TAG,"Sent data[%u]: 0x%x",i,out_data[i]);
  }

  err = i2c_dev_write(&dev->i2c_dev, NULL, 0, out_data, 4);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_WRITE;
    dev->sen.status.fail_time=esp_timer_get_time();
    dev->sen.status.fail_reg=ram_addr;
    ESP_LOGE(TAG, "Error on i2c_dev_write with error: %02x",err);
  }

  return ESP_OK;
  return err;
}

static inline esp_err_t read_resp(k30_t *dev, uint8_t *out_data, uint8_t data_size) {
  esp_err_t err=ESP_OK;

  err = i2c_dev_read(&dev->i2c_dev, NULL, 0, out_data, data_size+2);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_READ;
    dev->sen.status.fail_time=esp_timer_get_time();
    dev->sen.status.fail_cnt++;
    // dev->sen.status.fail_reg=ram_addr; //TODO: get last address written from status
    ESP_LOGE(TAG, "Error on i2c_dev_read with error: 0x%02x",err);
    return err;
  }
  if((k30_checksum(out_data, data_size+1)) != out_data[data_size+2-1]){
    ESP_LOGE(TAG, "Checksum check fail! checksum calc: 0x%02x, checksum received: 0x%02x",k30_checksum(out_data, data_size+1),out_data[data_size+2-1]);
    return ESP_ERR_INVALID_CRC;
  }
  return err;
}

// Initialization.
esp_err_t k30_init_desc(k30_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id, char *sen_name) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize descriptor");
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = K30_I2C_ADDR; // k30 has only one i2c address.
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,OUTPUT_NR);
    strcpy(dev->sen.info.name, sen_name);
    dev->sen.info.lib_id = SEN_K30_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.info.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.info.out_nr = OUTPUT_NR; //CO2, Temperature, RH
    dev->sen.conf.trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;

    dev->sen.conf.min_period_us = 10000000;
    dev->sen.conf.addr = K30_I2C_ADDR;
    dev->sen.conf.period_ms=CONFIG_K30_DEFAULT_PERIOD_MS;
    dev->sen.conf.srate=0;
    dev->sen.conf.time_to_adjust_us=0;

    dev->sen.esp_timestamp=0;
    dev->sen.dev=dev;
    dev->sen.reset=k30_iot_sen_reset;
    dev->sen.reinit=k30_iot_sen_reinit;
    dev->sen.start_measurement=k30_iot_sen_start_measurement;
    dev->sen.get_data=k30_iot_sen_get_data;
    dev->sen.awake=k30_iot_sen_sleep_mode_awake;
    dev->sen.gain_adjust=k30_iot_sen_start_measurement;
    dev->sen.sleep=k30_iot_sen_sleep_mode_sleep;

    dev->sen.conf.delay_start_get_us = 120000;
    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;

    dev->sen.outs[K30_OUT_CO2_ID].out_id=K30_OUT_CO2_ID;
    dev->sen.outs[K30_OUT_CO2_ID].out_val_type = SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[K30_OUT_CO2_ID].bit_nr=16;
    dev->sen.outs[K30_OUT_CO2_ID].m_raw=0;
    dev->sen.outs[K30_OUT_CO2_ID].processed=0.0;

    // dev->sen.outs[K30_OUT_TEMP_ID].out_id=K30_OUT_TEMP_ID;
    // dev->sen.outs[K30_OUT_TEMP_ID].out_val_type = SEN_OUT_VAL_TYPE_INT16;
    // dev->sen.outs[K30_OUT_TEMP_ID].bit_nr=16;
    // dev->sen.outs[K30_OUT_TEMP_ID].m_raw=0;
    // dev->sen.outs[K30_OUT_TEMP_ID].processed=0.0;
    //
    // dev->sen.outs[K30_OUT_RH_ID].out_id=K30_OUT_RH_ID;
    // dev->sen.outs[K30_OUT_RH_ID].out_val_type = SEN_OUT_VAL_TYPE_INT16;
    // dev->sen.outs[K30_OUT_RH_ID].bit_nr=16;
    // dev->sen.outs[K30_OUT_RH_ID].m_raw=0;
    // dev->sen.outs[K30_OUT_RH_ID].processed=0.0;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t k30_free_desc(k30_t *dev) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Free descriptor.");

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t k30_init(k30_t *dev) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize sensor.");

    //**** WARNING backup entire eeprom before any write!!!****//
    // Enable Dynamical frac algorithm if it is not enabled
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t out_data[1+4+1];
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_FW_TYPE, K30_COMMAND_READ_RAM, 1), "I2C error");
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    memset(out_data,0,1+4+1);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 1), "I2C error");
    dev->info.fw_type = out_data[1];
    ESP_LOGI(TAG,"fw_type: %u",dev->info.fw_type);
    vTaskDelay(pdMS_TO_TICKS(500));
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_REV_MAIN, K30_COMMAND_READ_RAM, 1), "I2C error");
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    memset(out_data,0,1+4+1);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 1), "I2C error");
    dev->info.rev_main = out_data[1];
    ESP_LOGI(TAG,"rev_main: %u",dev->info.rev_main);
    vTaskDelay(pdMS_TO_TICKS(500));
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_REV_SUB, K30_COMMAND_READ_RAM, 1), "I2C error");
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    memset(out_data,0,1+4+1);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 1), "I2C error");
    dev->info.rev_sub = out_data[1];
    ESP_LOGI(TAG,"rev_sub: %u",dev->info.rev_sub);
    vTaskDelay(pdMS_TO_TICKS(500));
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_SEN_TYPE, K30_COMMAND_READ_RAM, 3), "I2C error");
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    memset(out_data,0,1+4+1);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 3), "I2C error");
    dev->info.sen_type = (out_data[1]<<(3-1))|(out_data[2]<<(3-2))|out_data[3];
    ESP_LOGI(TAG,"sen_type: %u",dev->info.sen_type);
    vTaskDelay(pdMS_TO_TICKS(500));
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_SEN_SERIAL_NR, K30_COMMAND_READ_RAM, 4), "I2C error");
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    memset(out_data,0,1+4+1);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 4), "I2C error");
    dev->info.sen_serial_nr = (out_data[1]<<(4-1))|(out_data[2]<<(4-2))|(out_data[3]<<(4-3))|out_data[4];
    ESP_LOGI(TAG,"sen_serial_nr: %u",dev->info.sen_serial_nr);
    vTaskDelay(pdMS_TO_TICKS(500));
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_MEM_MAP_ID, K30_COMMAND_READ_RAM, 1), "I2C error");
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    memset(out_data,0,1+4+1);
    I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 1), "I2C error");
    dev->info.sen_mem_map_id = out_data[1];
    ESP_LOGI(TAG,"sen_mem_map_id: %u",dev->info.sen_mem_map_id);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    dev->sen.status.status_code=SEN_STATUS_OK;
    return ESP_OK;
}

esp_err_t k30_get_co2(k30_t *dev, float *co2) {
  CHECK_ARG(dev && co2);
  ESP_LOGD(TAG, "get CO2");
  uint8_t out_data[1+2+1];
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_CO2, K30_COMMAND_READ_RAM, 2),"Error writing RAM!");
  vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 2),"Error reading RAM!");

  // I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_ram(dev, , out_data, 2),"I2C error getting CO2");
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  dev->sen.esp_timestamp = esp_timer_get_time();
  dev->sen.outs[K30_OUT_CO2_ID].m_raw = (out_data[1]<<8)|out_data[2];
  dev->sen.outs[K30_OUT_CO2_ID].processed = (float)((int16_t)(dev->sen.outs[K30_OUT_CO2_ID].m_raw&0x0000ffff));
  *co2 = dev->sen.outs[K30_OUT_CO2_ID].processed;
  ESP_LOGI(TAG, "CO2: %f", *co2);
  return ESP_OK;
}

esp_err_t k30_get_temp(k30_t *dev, float *temp) {
  CHECK_ARG(dev && temp);
  ESP_LOGD(TAG, "get Temperature");
  uint8_t out_data[1+2+1];
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_TEMP, K30_COMMAND_READ_RAM, 2),"Error writing RAM!");
  vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 2),"Error reading RAM!");

  // I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_ram(dev, , out_data, 2),"I2C error getting CO2");
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  dev->sen.esp_timestamp = esp_timer_get_time();
  dev->sen.outs[K30_OUT_TEMP_ID].m_raw = (out_data[1]<<8)|out_data[2];
  dev->sen.outs[K30_OUT_TEMP_ID].processed = (float)((int16_t)(dev->sen.outs[K30_OUT_TEMP_ID].m_raw&0x0000ffff));
  *temp = dev->sen.outs[K30_OUT_TEMP_ID].processed;
  ESP_LOGI(TAG, "Temperature: %f", *temp);
  return ESP_OK;
}

esp_err_t k30_get_rh(k30_t *dev, float *rh) {
  CHECK_ARG(dev && rh);
  ESP_LOGD(TAG, "get RH");
  uint8_t out_data[1+2+1];
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK_LOGE(&dev->i2c_dev, write_ram(dev, K30_RAM_RH, K30_COMMAND_READ_RAM, 2),"Error writing RAM!");
  vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_resp(dev, out_data, 2),"Error reading RAM!");

  // I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_ram(dev, , out_data, 2),"I2C error getting CO2");
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  dev->sen.esp_timestamp = esp_timer_get_time();
  dev->sen.outs[K30_OUT_RH_ID].m_raw = (out_data[1]<<8)|out_data[2];
  dev->sen.outs[K30_OUT_RH_ID].processed = (float)((int16_t)(dev->sen.outs[K30_OUT_RH_ID].m_raw&0x0000ffff));
  dev->sen.outs[K30_OUT_RH_ID].processed = dev->sen.outs[K30_OUT_RH_ID].processed/100.00;
  *rh = dev->sen.outs[K30_OUT_RH_ID].processed;
  ESP_LOGI(TAG, "Relative humidity: %f", *rh);
  return ESP_OK;
}

esp_err_t k30_get_all_outputs(k30_t *dev, float *co2, float *temp, float *rh) {
  uint8_t out_data[1+2+1];
  esp_err_t err;
  bool onepass = false;
  CHECK_ARG(dev && co2 && temp && rh);
  ESP_LOGD(TAG, "get all outputs");

  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  err = write_ram(dev, K30_RAM_CO2, K30_COMMAND_READ_RAM, 2);
  if(err == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    err = read_resp(dev, out_data, 2);
  }
  if(err == ESP_OK && (out_data[0]&0x01)) {
    onepass = true;
    dev->sen.outs[K30_OUT_CO2_ID].m_raw = (out_data[1]<<8)|out_data[2];
    dev->sen.outs[K30_OUT_CO2_ID].processed = (float)((int16_t)(dev->sen.outs[K30_OUT_CO2_ID].m_raw&0x0000ffff));
    *co2 = dev->sen.outs[K30_OUT_CO2_ID].processed;
  }
  // vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  vTaskDelay(pdMS_TO_TICKS(500));

  err = write_ram(dev, K30_RAM_TEMP, K30_COMMAND_READ_RAM, 2);
  if(err == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    err = read_resp(dev, out_data, 2);
  }
  if(err == ESP_OK && (out_data[0]&0x01)) {
    onepass = true;
    dev->sen.outs[K30_OUT_TEMP_ID].m_raw = (out_data[1]<<8)|out_data[2];
    dev->sen.outs[K30_OUT_TEMP_ID].processed = (float)((int16_t)(dev->sen.outs[K30_OUT_TEMP_ID].m_raw&0x0000ffff));
    *temp = dev->sen.outs[K30_OUT_TEMP_ID].processed;
  }
  // vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
  vTaskDelay(pdMS_TO_TICKS(500));

  err = write_ram(dev, K30_RAM_RH, K30_COMMAND_READ_RAM, 2);
  if(err == ESP_OK) {
    vTaskDelay(pdMS_TO_TICKS(dev->sen.conf.delay_start_get_us/1000));
    err = read_resp(dev, out_data, 2);
  }
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  if(err == ESP_OK && (out_data[0]&0x01)) {
    onepass = true;
    dev->sen.outs[K30_OUT_RH_ID].m_raw = (out_data[1]<<8)|out_data[2];
    dev->sen.outs[K30_OUT_RH_ID].processed = (float)((int16_t)(dev->sen.outs[K30_OUT_RH_ID].m_raw&0x0000ffff));
    dev->sen.outs[K30_OUT_RH_ID].processed = dev->sen.outs[K30_OUT_RH_ID].processed/100.00;
    *rh = dev->sen.outs[K30_OUT_RH_ID].processed;
  }

  // I2C_DEV_CHECK_LOGE(&dev->i2c_dev, read_ram(dev, , out_data, 2),"I2C error getting CO2");
  if(onepass){
    dev->sen.esp_timestamp = esp_timer_get_time();
    ESP_LOGI(TAG, "CO2: %f", *co2);
    ESP_LOGI(TAG, "Temperature: %f", *temp);
    ESP_LOGI(TAG, "Relative humidity: %f", *rh);
  } else {
    ESP_LOGE(TAG, "No output was received with success!");
    return err;
  }

  return ESP_OK;
}

esp_err_t k30_basic_enable(k30_t *dev) {
  CHECK_ARG(dev);
  // float co2;
  // k30_get_co2((k30_t*) dev, &co2);
  // I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  //
  // // I2C_DEV_CHECK(&dev->i2c_dev,
  // //   write_enable_register(dev, dev->settings.enable_reg | K30_ALS_ON | K30_POWER_ON));
  // I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  return ESP_OK;
}

esp_err_t k30_basic_disable(k30_t *dev) {
  CHECK_ARG(dev);

  // I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  // I2C_DEV_CHECK(&dev->i2c_dev,
  //   write_enable_register(dev, dev->settings.enable_reg & ~K30_ALS_ON & ~K30_POWER_ON));

  return ESP_OK;
}

esp_err_t k30_iot_sen_start_measurement(void *dev) {
  // k30_t* dev_ = (k30_t*) dev;
  // I2C_DEV_TAKE_MUTEX(&dev_->i2c_dev);
  // I2C_DEV_CHECK_LOGE(&dev_->i2c_dev, write_ram(dev_, K30_RAM_CO2, K30_COMMAND_READ_RAM, 2),"Error writing RAM!");
  // I2C_DEV_GIVE_MUTEX(&dev_->i2c_dev);
  return ESP_OK;
}

esp_err_t k30_iot_sen_get_data(void *dev) {
  k30_t* dev_ = (k30_t*) dev;
  float co2, temp, rh;
  // esp_err_t ret = ESP_OK;
  // // uint16_t channel0, channel1;
  // uint8_t out_data[1+2+1];
  // // float co2, temp, rh;
  // I2C_DEV_TAKE_MUTEX(&dev_->i2c_dev);
  // I2C_DEV_CHECK_LOGE(&dev_->i2c_dev, read_resp(dev_, out_data, 2),"Error reading RAM!");
  //
  // // ret = k30_get_co2((k30_t*) dev, &co2);
  // dev_->sen.esp_timestamp = esp_timer_get_time();
  // dev_->sen.outs[K30_OUT_CO2_ID].m_raw = (out_data[1]<<1)|out_data[2];
  // dev_->sen.outs[K30_OUT_CO2_ID].processed = (float)((int16_t)(dev_->sen.outs[K30_OUT_CO2_ID].m_raw&0x0000ffff));
  // I2C_DEV_GIVE_MUTEX(&dev_->i2c_dev);
  // return k30_get_all_outputs((k30_t*) dev, &co2, &temp, &rh);
  return k30_get_co2((k30_t*) dev, &co2);
  // return ret;
}

esp_err_t k30_iot_sen_sleep_mode_awake(void *dev) {
  k30_t* dev_ = (k30_t*) dev;
  return k30_basic_enable(dev_);
}

esp_err_t k30_iot_sen_sleep_mode_sleep(void *dev) {
  k30_t* dev_ = (k30_t*) dev;
  return k30_basic_disable(dev_);
}

esp_err_t k30_iot_sen_reset(void *dev) {
  return ESP_OK;
}

esp_err_t k30_iot_sen_reinit(void *dev) {
  return k30_init((k30_t*) dev);
}
