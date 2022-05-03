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

#define I2C_FREQ_HZ 100000 // 100kHz

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
  K30_COMMAND_WRITE_RAM =    0x1,
  K30_COMMAND_READ_RAM =     0x2,
  K30_COMMAND_WRITE_EEPROM = 0x3,
  K30_COMMAND_READ_EEPROM =  0x4
} k30_command_t;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)


static inline uint8_t k30_checksum(uint8_t data[], size_t len) {
  uint8_t checksum = 0x00;
  size_t i;

  for (i = 0; i < len; i++) {
    ESP_LOGD(TAG,"data[%u]: %02x",i,data[i]);
    checksum += data[i];
  }
  ESP_LOGD(TAG,"checksum: %02x",checksum);
  return checksum;
}

// Read/write to registers.
static inline esp_err_t write_ram(k30_t *dev, uint16_t ram_addr, uint8_t *data, uint8_t data_size) {
  esp_err_t err=ESP_OK;
  uint8_t out_reg[3];
  uint8_t out_data[data_size+1];

  out_reg[0]=((K30_COMMAND_WRITE_RAM<<1) | data_size);
  out_reg[1]=((ram_addr>>1) & 0xFF);
  out_reg[2]=(ram_addr & 0xFF);
  memcpy(out_data,data,data_size);
  out_data[data_size]=k30_checksum(out_reg, 3);
  out_data[data_size]+=k30_checksum(data, data_size);
  ESP_LOGD(TAG, "Writing ram address: 0x%x", ram_addr);
  ESP_LOGD(TAG, "Writing ram address_0: 0x%x", out_data[1]);
  ESP_LOGD(TAG, "Writing ram address_1: 0x%x", out_data[2]);
  err = i2c_dev_write(&dev->i2c_dev, out_reg, 3, out_data, data_size+1);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_WRITE;
    dev->sen.status.fail_reg=ram_addr;
  }
  return err;
}

static inline esp_err_t read_ram(k30_t *dev, uint16_t ram_addr, uint8_t *out_data, uint8_t data_size) {
  esp_err_t err=ESP_OK;
  uint8_t out_reg[3];

  out_reg[0]=((K30_COMMAND_READ_RAM<<1) | (data_size-1));
  out_reg[1]=((ram_addr>>1) & 0xFF);
  out_reg[2]=(ram_addr & 0xFF);
  err = i2c_dev_read(&dev->i2c_dev, out_data, data_size, out_reg, 3);
  if(err != ESP_OK){
    dev->sen.status.status_code=SEN_STATUS_FAIL_WRITE;
    dev->sen.status.fail_reg=ram_addr;
  }
  if(k30_checksum(out_data, data_size-1) != out_data[data_size-1]){
    ESP_LOGE(TAG, "Checksum check fail! checksum calc: %u, checksum received: %u",k30_checksum(out_data, data_size-1),out_data[data_size-1]);
    return ESP_ERR_INVALID_CRC;
  }
  return err;
}

// Read 16 bit from two consecutive registers.
// Note that the sensor will shadow for example C0DATAH if C0DATAL is read.
// static inline esp_err_t read_register16(k30_t *dev, uint8_t low_register, uint16_t *value) {
//     uint8_t buf[2];
//     esp_err_t err=ESP_OK;
//     err = i2c_dev_read_reg(&dev->i2c_dev,
//         K30_RAM_COMMAND | K30_TRANSACTION_NORMAL | low_register, buf, 2);
//     if(err != ESP_OK){
//       dev->sen.status.status_code=SEN_STATUS_FAIL_READ;
//       dev->sen.status.fail_reg=low_register;
//     }
//     *value = (uint16_t)buf[1] << 8 | buf[0];
//
//     return err;
// }

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
    sensor_init(&dev->sen,3);
    // strncpy(dev->sen.info.name, sen_name, strlen(sen_name));
    strcpy(dev->sen.info.name, sen_name);
    // strncpy(dev->sen.info.name, "K30\0", 8);
    dev->sen.info.lib_id = SEN_K30_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.conf.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.info.out_nr = 3; //CO2, Temperature, RH
    dev->sen.info.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;

    dev->sen.conf.min_period_us = 1000000;
    dev->sen.conf.addr = K30_I2C_ADDR;
    dev->sen.conf.period_ms=nearest_prime(CONFIG_K30_DEFAULT_PERIOD_MS);
    dev->sen.conf.srate=0;

    dev->sen.timestamp=0;
    dev->sen.dev=dev;
    dev->sen.reset=k30_iot_sen_reset;
    dev->sen.reinit=k30_iot_sen_reinit;
    dev->sen.start_measurement=k30_iot_sen_start_measurement;
    dev->sen.get_data=k30_iot_sen_get_data;
    dev->sen.awake=k30_iot_sen_sleep_mode_awake;
    dev->sen.gain_adjust=k30_iot_sen_start_measurement;
    dev->sen.sleep=k30_iot_sen_sleep_mode_sleep;

    dev->sen.conf.delay_start_get_us = 120000;
    dev->sen.status.initialized = false;
    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;

    dev->sen.outs[K30_OUT_CO2_ID].out_id=K30_OUT_CO2_ID;
    dev->sen.outs[K30_OUT_CO2_ID].out_type = SEN_TYPE_CO2;
    dev->sen.outs[K30_OUT_CO2_ID].out_val_type = SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[K30_OUT_CO2_ID].bit_nr=16;
    dev->sen.outs[K30_OUT_CO2_ID].m_raw=0;
    dev->sen.outs[K30_OUT_CO2_ID].co2=0.0;

    dev->sen.outs[K30_OUT_TEMP_ID].out_id=K30_OUT_TEMP_ID;
    dev->sen.outs[K30_OUT_TEMP_ID].out_type = SEN_TYPE_AMBIENT_TEMPERATURE;
    dev->sen.outs[K30_OUT_TEMP_ID].out_val_type = SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[K30_OUT_TEMP_ID].bit_nr=16;
    dev->sen.outs[K30_OUT_TEMP_ID].m_raw=0;
    dev->sen.outs[K30_OUT_TEMP_ID].temperature=0.0;

    dev->sen.outs[K30_OUT_RH_ID].out_id=K30_OUT_TEMP_ID;
    dev->sen.outs[K30_OUT_RH_ID].out_type = SEN_TYPE_RELATIVE_HUMIDITY;
    dev->sen.outs[K30_OUT_RH_ID].out_val_type = SEN_OUT_VAL_TYPE_INT16;
    dev->sen.outs[K30_OUT_RH_ID].bit_nr=16;
    dev->sen.outs[K30_OUT_RH_ID].m_raw=0;
    dev->sen.outs[K30_OUT_RH_ID].relative_humidity=0.0;

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
    I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_FW_TYPE, out_data, 1+1));
    dev->info.fw_type = out_data[1];
    ESP_LOGD(TAG,"fw_type: %u",dev->info.fw_type);
    I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_REV_MAIN, out_data, 1+1));
    dev->info.rev_main = out_data[1];
    ESP_LOGD(TAG,"rev_main: %u",dev->info.rev_main);
    I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_REV_SUB, out_data, 1+1));
    dev->info.rev_sub = out_data[1];
    ESP_LOGD(TAG,"rev_sub: %u",dev->info.rev_sub);
    I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_SEN_TYPE, out_data, 3+1));
    dev->info.sen_type = (out_data[1]<<(3-1))|(out_data[2]<<(3-2))|out_data[3];
    ESP_LOGD(TAG,"sen_type: %u",dev->info.sen_type);
    I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_SEN_SERIAL_NR, out_data, 4+1));
    dev->info.sen_serial_nr = (out_data[1]<<(4-1))|(out_data[2]<<(4-2))|(out_data[3]<<(4-3))|out_data[4];
    ESP_LOGD(TAG,"sen_serial_nr: %u",dev->info.sen_serial_nr);
    I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_MEM_MAP_ID, out_data, 1+1));
    dev->info.sen_mem_map_id = out_data[1];
    ESP_LOGD(TAG,"sen_mem_map_id: %u",dev->info.sen_mem_map_id);






    // I2C_DEV_CHECK(&dev->i2c_dev, read_enable_register(dev, &tmp_reg));
    //
    // ESP_LOGD(TAG, "Initial enable register: %x.", tmp_reg);
    //
    // I2C_DEV_CHECK(&dev->i2c_dev, read_control_register(dev, &tmp_reg));
    // dev->settings.control_reg = tmp_reg;
    // ESP_LOGD(TAG, "Initial control register: %x.", tmp_reg);
    // ESP_LOGD(TAG, "Gain: %x.", (tmp_reg & 0x30));
    // ESP_LOGD(TAG, "Integration time: %x.", (tmp_reg & 0x07));
    //
    // I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, K30_RAM_PERSIST, &tmp_reg));
    // dev->settings.persistence_reg = tmp_reg;
    // ESP_LOGD(TAG, "Initial persistence filter: %x.", tmp_reg);
    //
    // I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, K30_RAMISTER_PACKAGE_PID, &tmp_reg));
    // dev->info.pack_id = tmp_reg;
    // ESP_LOGD(TAG, "Package ID: %x.", tmp_reg);
    //
    // I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, K30_RAMISTER_DEVICE_ID, &tmp_reg));
    // dev->info.dev_id = tmp_reg;
    // ESP_LOGD(TAG, "Device ID: %x.", tmp_reg);
    //
    // I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, K30_RAMISTER_DEVICE_STATUS, &tmp_reg));
    // dev->info.status = tmp_reg;
    // ESP_LOGD(TAG, "Device status: %x.", tmp_reg);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    // if(dev->info.dev_id!=0x50) {
    //   ESP_LOGE(TAG, "Device ID: %x differs from expected ID: %x", tmp_reg,0x50);
    //   dev->sen.status.initialized = false;
    //   dev->sen.status.fail_cnt++;
    //   dev->sen.status.fail_reg = K30_RAMISTER_DEVICE_ID;
    //   time(&dev->sen.status.fail_time);
    //   dev->sen.status.status_code=SEN_STATUS_FAIL_CHECKSUM;
    //   return ESP_FAIL;
    // }

    // k30_basic_disable(dev);
    dev->sen.status.initialized=true;
    dev->sen.status.status_code=SEN_STATUS_OK;
    return ESP_OK;
}

esp_err_t k30_get_co2(k30_t *dev, float *co2) {
  CHECK_ARG(dev && co2);
  uint8_t out_data[1+2+1];
  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, read_ram(dev, K30_RAM_CO2, out_data, 1+2+1));
  dev->sen.timestamp = esp_timer_get_time();
  dev->sen.outs[K30_OUT_CO2_ID].m_raw = (out_data[1]<<1)|out_data[2];
  dev->sen.outs[K30_OUT_CO2_ID].co2 = (float)((int16_t)(dev->sen.outs[K30_OUT_CO2_ID].m_raw&0x0000ffff));
  *co2 = dev->sen.outs[K30_OUT_CO2_ID].co2;
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
  return ESP_OK;
}

esp_err_t k30_basic_enable(k30_t *dev) {
  CHECK_ARG(dev);
  float co2;
  k30_get_co2((k30_t*) dev, &co2);
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
  return ESP_OK;
}

esp_err_t k30_iot_sen_get_data(void *dev) {
  esp_err_t ret = ESP_OK;
  // uint16_t channel0, channel1;
  float co2, temp, rh;

  ret = k30_get_co2((k30_t*) dev, &co2);
  return ret;
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
  return ESP_OK;
}
