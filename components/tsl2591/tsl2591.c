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
 * @file tsl2591.c
 *
 * ESP-IDF driver for TSL2591 light-to-digital.
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
#include <string.h>
#include <sys/time.h>
#include "tsl2591.h"

#define I2C_FREQ_HZ 80000 // 80kHz

static const char *TAG = "tsl2591";

// Registers
#define TSL2591_REG_COMMAND 0x80
#define TSL2591_REG_ENABLE  0x00
#define TSL2591_REG_CONTROL 0x01
#define TSL2591_REG_AILTL   0x04    // ALS interrupt low threshold low byte
#define TSL2591_REG_AILTH   0x05    // ALS interrupt low threshold high byte
#define TSL2591_REG_AIHTL   0x06    // ALS interrupt high threshold low byte
#define TSL2591_REG_AIHTH   0x07    // ALS interrupt high threshold high byte
#define TSL2591_REG_NPAILTL 0x08    // No ALS persist interrupt low threshold low byte
#define TSL2591_REG_NPAILTH 0x09    // No ALS persist interrupt low threshold high byte
#define TSL2591_REG_NPAIHTL 0x0A    // No ALS persist interrupt high threshold low byte
#define TSL2591_REG_NPAIHTH 0x0B    // No ALS persist interrupt high threshold high byte
#define TSL2591_REG_PERSIST 0x0C    // Interrupt persistence filter
#define TSL2591_REG_C0DATAL 0x14
#define TSL2591_REG_C0DATAH 0x15
#define TSL2591_REG_C1DATAL 0x16
#define TSL2591_REG_C1DATAH 0x17
#define TSL2591_REG_STATUS  0x13

// TSL2591 command register transaction mode.
#define TSL2591_TRANSACTION_NORMAL  0x20    // Normal transaction for addressing registers
#define TSL2591_TRANSACTION_SPECIAL 0x60    // Special transactions for interrupt clearing

// TSL2591 command register special functions.
#define TSL2591_SPECIAL_SET_INTR        0x04    // Forces interrupt
#define TSL2591_SPECIAL_CLEAR_INTR      0x06    // Clear ALS interrupt
#define TSL2591_SPECIAL_CLEAR_BOTH      0x07    // Clear ALS and no persist ALS interrupt
#define TSL2591_SPECIAL_CLEAR_NP_INTR   0x0A    // Clear no persist ALS interrupt

// TSL2591 integration times in useconds.
#define TSL2591_INTEGRATION_TIME_100MS  110
#define TSL2591_INTEGRATION_TIME_200MS  210
#define TSL2591_INTEGRATION_TIME_300MS  310
#define TSL2591_INTEGRATION_TIME_400MS  410
#define TSL2591_INTEGRATION_TIME_500MS  510
#define TSL2591_INTEGRATION_TIME_600MS  610

// TSL2591 status flags.
#define TSL2591_STATUS_ALS_INTR     0x10
#define TSL2591_STATUS_ALS_NP_INTR  0x20
#define TSL2591_STATUS_ALS_VALID    0x01

// Calculation constants.
#define TSL2591_LUX_DF 408.0F   //< Lux cooefficient
#define TSL2591_LUX_COEFB 1.64F //< CH0 coefficient
#define TSL2591_LUX_COEFC 0.59F //< CH1 coefficient A
#define TSL2591_LUX_COEFD 0.86F //< CH2 coefficient B

#define TSL2591_GAINS_NR 4
#define TSL2591_INTEGRATION_TIMES_NR 6



#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

const uint32_t gains[TSL2591_GAINS_NR] = {1,25,428,9876};
const uint32_t gains_max_values[TSL2591_GAINS_NR] = {65535,65535,65535,65535};
const int32_t gains_min_values[TSL2591_GAINS_NR] = {0,0,0,0};
const float gains_th_h[TSL2591_GAINS_NR] = {0.90,0.90,0.90,0.90};
const float gains_th_l[TSL2591_GAINS_NR] = {0.03,0.03,0.03,0.03};
const uint32_t int_times[TSL2591_INTEGRATION_TIMES_NR] = {100,200,300,400,500,600};
const uint32_t itimes_max_values[TSL2591_INTEGRATION_TIMES_NR] = {37888,65535,65535,65535,65535,65535};
const int32_t itimes_min_values[TSL2591_INTEGRATION_TIMES_NR] = {0,0,0,0,0,0};
const float itimes_th_h[TSL2591_INTEGRATION_TIMES_NR] = {0.70,0.75,0.80,0.85,0.90,0.95};
const float itimes_th_l[TSL2591_INTEGRATION_TIMES_NR] = {0.03,0.03,0.03,0.03,0.03,0.03};


// Read/write to registers.
static inline esp_err_t write_register(tsl2591_t *dev, uint8_t reg, uint8_t value)
{
    ESP_LOGD(TAG, "Writing register: 0x%x; Data: 0x%x.", reg, value);
    return i2c_dev_write_reg(&dev->i2c_dev,
        TSL2591_REG_COMMAND | TSL2591_TRANSACTION_NORMAL | reg, &value, 1);
}

static inline esp_err_t read_register(tsl2591_t *dev, uint8_t reg, uint8_t *value)
{
    esp_err_t err;
    err = i2c_dev_read_reg(&dev->i2c_dev,
        TSL2591_REG_COMMAND | TSL2591_TRANSACTION_NORMAL | reg, value, 1);
    ESP_LOGD(TAG, "Red register: 0x%x; Data: 0x%x.", reg, *value);
    return err;
}

// Write special function to command register.
static inline esp_err_t write_special_function(tsl2591_t *dev, uint8_t special_function)
{
    uint8_t function = TSL2591_REG_COMMAND | TSL2591_TRANSACTION_SPECIAL | special_function;
    ESP_LOGD(TAG, "Calling special function: 0x%x with 0x%x.", special_function, function);
    return i2c_dev_write(&dev->i2c_dev, NULL, 0, &function, 8);
}

// Read/write enable register.
static inline esp_err_t write_enable_register(tsl2591_t *dev, uint8_t value)
{
    return write_register(dev, TSL2591_REG_ENABLE, value);
}

static inline esp_err_t read_enable_register(tsl2591_t *dev, uint8_t *value)
{
    return read_register(dev, TSL2591_REG_ENABLE, value);
}

// Read/write control register.
static inline esp_err_t write_control_register(tsl2591_t *dev, uint8_t value)
{
    return write_register(dev, TSL2591_REG_CONTROL, value);
}

static inline esp_err_t read_control_register(tsl2591_t *dev, uint8_t *value)
{
    return read_register(dev, TSL2591_REG_CONTROL, value);
}

// Read 16 bit from two consecutive registers.
// Note that the sensor will shadow for example C0DATAH if C0DATAL is read.
static inline esp_err_t read_register16(tsl2591_t *dev, uint8_t low_register, uint16_t *value)
{
    uint8_t buf[2];
    CHECK(i2c_dev_read_reg(&dev->i2c_dev,
        TSL2591_REG_COMMAND | TSL2591_TRANSACTION_NORMAL | low_register, buf, 2));
    *value = (uint16_t)buf[1] << 8 | buf[0];

    return ESP_OK;
}

static inline tsl2591_gain_t idx2gain(uint8_t idx)
{
  switch(idx)
  {
    case 0:
      return TSL2591_GAIN_LOW;
    break;
    case 1:
      return TSL2591_GAIN_MEDIUM;
    break;
    case 2:
      return TSL2591_GAIN_HIGH;
    break;
    case 3:
      return TSL2591_GAIN_MAX;
    break;
    default:
      return TSL2591_GAIN_LOW;
    break;
  }
}

static inline tsl2591_integration_time_t idx2itime(uint8_t idx)
{
  switch(idx)
  {
    case 0:
      return TSL2591_INTEGRATION_100MS;
    break;
    case 1:
      return TSL2591_INTEGRATION_200MS;
    break;
    case 2:
      return TSL2591_INTEGRATION_300MS;
    break;
    case 3:
      return TSL2591_INTEGRATION_400MS;
    break;
    case 4:
      return TSL2591_INTEGRATION_500MS;
    break;
    case 5:
      return TSL2591_INTEGRATION_600MS;
    break;
    default:
      return TSL2591_INTEGRATION_100MS;
    break;
  }
}


// Initialization.
esp_err_t tsl2591_init_desc(tsl2591_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id)
{
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize descriptor");
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = TSL2591_I2C_ADDR; // tsl2591 has only one i2c address.
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,2);
    strncpy(dev->sen.name, "TSL2591\0", 8);
    dev->sen.lib_id = SEN_TSL2591_LIB_ID;
    dev->sen.sen_id = sen_id;
    dev->sen.version = 1;
    dev->sen.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.min_period_us = 0;
    dev->sen.delay_s_ms = 0;
    dev->sen.out_nr = 2; //channel 0 (Full spectrum) and channel 1 (IR)
    dev->sen.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.addr = TSL2591_I2C_ADDR;
    dev->sen.timestamp=0;
    dev->sen.period_ms=5420;
    dev->sen.get_data=tsl2591_iot_sen_measurement;
    dev->sen.dev=dev;

    dev->sen.sen_status.initialized = false;
    dev->sen.sen_status.fail_cnt = 0;
    dev->sen.sen_status.fail_time = 0;

    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].out_id=TSL2591_OUT_CH0_ID;
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].out_type = SEN_TYPE_LIGHT;
    // dev->sen.sen_outs[TSL2591_OUT_CH0_ID].sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].out_val_type = SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].bit_nr=16;
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].measurement_raw=0;
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].light=0.0;
    // dev->sen.sen_outs[TSL2591_OUT_CH0_ID].timestamp=0;
    ESP_ERROR_CHECK(sensor_out_agc_init(&dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc, SEN_AGC_TYPE_GAIN, TSL2591_GAINS_NR, gains, gains_max_values,gains_min_values,gains_th_h,gains_th_l));
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.state = true;
    ESP_ERROR_CHECK(sensor_out_agc_init(&dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc, SEN_AGC_TYPE_NONE, TSL2591_INTEGRATION_TIMES_NR, int_times, itimes_max_values,itimes_min_values,itimes_th_h,itimes_th_l));
    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc.state = true;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].out_id=TSL2591_OUT_CH1_ID;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].out_type = SEN_TYPE_LIGHT;
    // dev->sen.sen_outs[TSL2591_OUT_CH1_ID].sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].out_val_type = SEN_OUT_VAL_TYPE_UINT16;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].bit_nr=16;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].measurement_raw=0;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].light=0.0;
    // dev->sen.sen_outs[TSL2591_OUT_CH1_ID].timestamp=0;
    ESP_ERROR_CHECK(sensor_out_agc_init(&dev->sen.sen_outs[TSL2591_OUT_CH1_ID].gains_agc, SEN_AGC_TYPE_GAIN,TSL2591_GAINS_NR, gains, gains_max_values,gains_min_values,gains_th_h,gains_th_l));
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].gains_agc.state = true;
    ESP_ERROR_CHECK(sensor_out_agc_init(&dev->sen.sen_outs[TSL2591_OUT_CH1_ID].itimes_agc, SEN_AGC_TYPE_NONE, TSL2591_INTEGRATION_TIMES_NR, int_times, itimes_max_values,itimes_min_values,itimes_th_h,itimes_th_l));
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].itimes_agc.state = true;

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t tsl2591_free_desc(tsl2591_t *dev)
{
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Free descriptor.");

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t tsl2591_init(tsl2591_t *dev)
{
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize sensor.");

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t tmp_reg = 0;
    I2C_DEV_CHECK(&dev->i2c_dev, read_enable_register(dev, &tmp_reg));
    dev->settings.enable_reg = tmp_reg;
    ESP_LOGD(TAG, "Initial enable register: %x.", tmp_reg);

    I2C_DEV_CHECK(&dev->i2c_dev, read_control_register(dev, &tmp_reg));
    dev->settings.control_reg = tmp_reg;
    ESP_LOGD(TAG, "Initial control register: %x.", tmp_reg);
    ESP_LOGD(TAG, "Gain: %x.", (tmp_reg & 0x30));
    ESP_LOGD(TAG, "Integration time: %x.", (tmp_reg & 0x07));

    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2591_REG_PERSIST, &tmp_reg));
    dev->settings.persistence_reg = tmp_reg;
    ESP_LOGD(TAG, "Initial persistence filter: %x.", tmp_reg);

    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2591_REGISTER_PACKAGE_PID, &tmp_reg));
    dev->info.pack_id = tmp_reg;
    ESP_LOGD(TAG, "Package ID: %x.", tmp_reg);

    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2591_REGISTER_DEVICE_ID, &tmp_reg));
    dev->info.dev_id = tmp_reg;
    ESP_LOGD(TAG, "Device ID: %x.", tmp_reg);

    I2C_DEV_CHECK(&dev->i2c_dev, read_register(dev, TSL2591_REGISTER_DEVICE_STATUS, &tmp_reg));
    dev->info.status = tmp_reg;
    ESP_LOGD(TAG, "Device status: %x.", tmp_reg);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if(dev->info.dev_id!=0x50)
    {
      ESP_LOGE(TAG, "Device ID: %x differs from expected ID: %x", tmp_reg,0x50);
      dev->sen.sen_status.initialized = false;
      dev->sen.sen_status.fail_cnt++;
      dev->sen.sen_status.fail_reg = TSL2591_REGISTER_DEVICE_ID;
      time(&dev->sen.sen_status.fail_time);
      return ESP_FAIL;
    }

    tsl2591_basic_disable(dev);
    dev->sen.sen_status.initialized=true;
    dev->sen.sen_status.status_code = SEN_STATUS_OK;
    dev->sen.sen_lib_version = TSL2591_LIB_VERSION;
    // Wait until the first integration cycle is completed.
    tsl2591_integration_time_t integration_time;
    ESP_ERROR_CHECK(tsl2591_get_integration_time(dev, &integration_time));
    switch (integration_time)
    {
    case TSL2591_INTEGRATION_100MS:
        SLEEP_MS(110);
        break;
    case TSL2591_INTEGRATION_200MS:
        SLEEP_MS(210);
        break;
    case TSL2591_INTEGRATION_300MS:
        SLEEP_MS(310);
        break;
    case TSL2591_INTEGRATION_400MS:
        SLEEP_MS(410);
        break;
    case TSL2591_INTEGRATION_500MS:
        SLEEP_MS(510);
        break;
    case TSL2591_INTEGRATION_600MS:
        SLEEP_MS(610);
        break;
    }

    return ESP_OK;
}

esp_err_t tsl2591_iot_sen_measurement(void *dev) {
  // esp_err_t ret;
  // uint16_t channel0, channel1;
  float lux;
  // ret = tsl2591_get_channel_data((tsl2591_t*) dev, &channel0, &channel1);
  // if(ret!=ESP_OK) return ret;
  // return tsl2591_calculate_lux();
  return tsl2591_get_lux((tsl2591_t*) dev,&lux);
}

// Measure.
esp_err_t tsl2591_get_channel_data(tsl2591_t *dev, uint16_t *channel0, uint16_t *channel1)
{
    // uint64_t timestamp;
    struct timeval tv;
    CHECK_ARG(dev && channel0 &&  channel1);
    if((dev->settings.enable_reg & (TSL2591_POWER_ON | TSL2591_ALS_ON)) != (TSL2591_POWER_ON | TSL2591_ALS_ON))
      tsl2591_basic_enable(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev, read_register16(dev, TSL2591_REG_C0DATAL, channel0));
    I2C_DEV_CHECK(&dev->i2c_dev, read_register16(dev, TSL2591_REG_C1DATAL, channel1));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  	gettimeofday(&tv, NULL);
  	// timestamp = (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL)) ;
  	// timestamp = tv.tv_sec * 1000000LL + tv.tv_usec;
    dev->sen.timestamp = tv.tv_sec * 1000000LL + tv.tv_usec;
    sen_agc_change_type_t agc_change = SEN_AGC_CHANGE_NOP;
    ESP_LOGD(TAG,"channel0: %u",*channel0);
    ESP_LOGD(TAG,"channel1: %u",*channel1);
    agc_change |= sensor_out_agc_change(dev->sen.sen_outs[TSL2591_OUT_CH0_ID], (uint32_t)*channel0);
    agc_change |= sensor_out_agc_change(dev->sen.sen_outs[TSL2591_OUT_CH1_ID], (uint32_t)*channel1);
    ESP_LOGD(TAG,"agc_change: %u",agc_change);
    if(agc_change==SEN_AGC_CHANGE_UP)
    {
      if(dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.idx < dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.val_nr-1)
      {
        ESP_LOGD(TAG,"Gain too low. Adjusting gain UP...");
        tsl2591_set_gain(dev,idx2gain(dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.idx+1));
        vTaskDelay(pdMS_TO_TICKS(dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc.values[dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc.idx]+20));
        tsl2591_get_channel_data(dev, channel0, channel1);
      }
    }
    else if(agc_change==SEN_AGC_CHANGE_DOWN)
    {
      if(dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.idx > 0)
      {
        ESP_LOGD(TAG,"Gain too high. Adjusting gain DOWN...");
        tsl2591_set_gain(dev,idx2gain(dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.idx-1));
        vTaskDelay(pdMS_TO_TICKS(dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc.values[dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc.idx]+20));
        tsl2591_get_channel_data(dev, channel0, channel1);
      }
      else
      {
        ESP_LOGW(TAG,"Sensor saturated!");
      }
    }
    // else
    // {
    tsl2591_basic_disable(dev);

    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].measurement_raw = *channel0;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].measurement_raw = *channel1;
    // dev->sen.sen_outs[TSL2591_OUT_CH0_ID].timestamp = dev->sen.sen_outs[TSL2591_OUT_CH1_ID].timestamp = timestamp;

    return ESP_OK;
}

esp_err_t tsl2591_calculate_lux(tsl2591_t *dev, uint16_t channel0, uint16_t channel1, float *lux)
{
    CHECK_ARG(dev && lux);

    float atime, again;
    switch (dev->settings.control_reg & 0x07)
    {
    case TSL2591_INTEGRATION_100MS:
        atime = int_times[0];
        break;
    case TSL2591_INTEGRATION_200MS:
        atime = int_times[1];
        break;
    case TSL2591_INTEGRATION_300MS:
        atime = int_times[2];
        break;
    case TSL2591_INTEGRATION_400MS:
        atime = int_times[3];
        break;
    case TSL2591_INTEGRATION_500MS:
        atime = int_times[4];
        break;
    case TSL2591_INTEGRATION_600MS:
        atime = int_times[5];
        break;
    default:
        atime = int_times[0];
    }

    switch (dev->settings.control_reg & TSL2591_GAIN_MAX)
    {
    case TSL2591_GAIN_LOW:
        again = gains[0];
        break;
    case TSL2591_GAIN_MEDIUM:
        again = gains[1];
        break;
    case TSL2591_GAIN_HIGH:
        again = gains[2];
        break;
    case TSL2591_GAIN_MAX:
        again = gains[3];
        break;
    default:
        again = gains[0];
    }
    float cpl = (atime * again) / TSL2591_LUX_DF;
    *lux = (((float)channel0 - (float)channel1)) *
        (1.0F - ((float)channel1 / (float)channel0)) / cpl;
    dev->sen.sen_outs[0].light = dev->sen.sen_outs[1].light = *lux; //TODO: store processed data in separate chanel?
    return ESP_OK;
}

esp_err_t tsl2591_get_lux(tsl2591_t *dev, float *lux)
{
    esp_err_t err;
    CHECK_ARG(dev && lux);

    uint16_t channel0, channel1;
    err = tsl2591_get_channel_data(dev, &channel0, &channel1);
    CHECK(err);

    err = tsl2591_calculate_lux(dev, channel0, channel1, lux);
    CHECK(err);

    return err;
}


// Setters and getters enable register.
esp_err_t tsl2591_set_power_status(tsl2591_t *dev, tsl2591_power_status_t power_status)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_enable_register(dev, (dev->settings.enable_reg & ~TSL2591_POWER_ON) | power_status));
    dev->settings.enable_reg = (dev->settings.enable_reg & ~TSL2591_POWER_ON) | power_status;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_power_status(tsl2591_t *dev, tsl2591_power_status_t *power_status)
{
    CHECK_ARG(dev && power_status);

    *power_status = dev->settings.enable_reg & TSL2591_POWER_ON;

    return ESP_OK;
}

esp_err_t tsl2591_set_als_status(tsl2591_t *dev, tsl2591_als_status_t als_status)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_enable_register(dev, (dev->settings.enable_reg & ~TSL2591_ALS_ON) | als_status));
    dev->settings.enable_reg = (dev->settings.enable_reg & ~TSL2591_ALS_ON) | als_status;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_als_status(tsl2591_t *dev, tsl2591_als_status_t *als_status)
{
    CHECK_ARG(dev && als_status);

    *als_status = dev->settings.enable_reg & TSL2591_ALS_ON;

    return ESP_OK;
}

esp_err_t tsl2591_set_interrupt(tsl2591_t *dev, tsl2591_interrupt_t interrupt)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_enable_register(dev, (dev->settings.enable_reg & ~TSL2591_ALS_INTR_BOTH_ON) | interrupt));
    dev->settings.enable_reg = (dev->settings.enable_reg & ~TSL2591_ALS_INTR_BOTH_ON) | interrupt;

    uint8_t tmp = 0;
    I2C_DEV_CHECK(&dev->i2c_dev,
        read_enable_register(dev, &tmp));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_interrupt(tsl2591_t *dev, tsl2591_interrupt_t *interrupt)
{
    CHECK_ARG(dev && interrupt);

    *interrupt = dev->settings.enable_reg & TSL2591_ALS_INTR_BOTH_ON;

    return ESP_OK;
}

esp_err_t tsl2591_set_sleep_after_intr(tsl2591_t *dev, tsl2591_sleep_after_intr_t sleep_after_intr)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_enable_register(dev, (dev->settings.enable_reg & ~TSL2591_SLEEP_AFTER_ON) | sleep_after_intr));
    dev->settings.enable_reg = (dev->settings.enable_reg & ~TSL2591_SLEEP_AFTER_ON) | sleep_after_intr;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_sleep_after_intr(tsl2591_t *dev, tsl2591_sleep_after_intr_t *sleep_after_intr)
{
    CHECK_ARG(dev && sleep_after_intr);

    *sleep_after_intr = dev->settings.enable_reg & TSL2591_SLEEP_AFTER_ON;

    return ESP_OK;
}


// Setters and getters control register.
esp_err_t tsl2591_set_integration_time(tsl2591_t *dev, tsl2591_integration_time_t integration_time)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    // Last 3 bits represent the integration time.
    I2C_DEV_CHECK(&dev->i2c_dev,
        write_control_register(dev, (dev->settings.control_reg & ~0x07) | integration_time));
    dev->settings.control_reg = (dev->settings.control_reg & ~0x07) | integration_time;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    uint8_t i_idx;
    switch(integration_time)
    {
      case TSL2591_INTEGRATION_100MS:
        i_idx=0;
      break;
      case TSL2591_INTEGRATION_200MS:
        i_idx=1;
      break;
      case TSL2591_INTEGRATION_300MS:
        i_idx=2;
      break;
      case TSL2591_INTEGRATION_400MS:
        i_idx=3;
      break;
      case TSL2591_INTEGRATION_500MS:
        i_idx=4;
      break;
      case TSL2591_INTEGRATION_600MS:
        i_idx=5;
      break;
      default:
        i_idx=0;
      break;
    }

    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].itimes_agc.idx=i_idx;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].itimes_agc.idx=i_idx;

    return ESP_OK;
}

esp_err_t tsl2591_get_integration_time(tsl2591_t *dev, tsl2591_integration_time_t *integration_time)
{
    CHECK_ARG(dev && integration_time);

    // Last 3 bits represent the integration time.
    *integration_time = dev->settings.control_reg & 0x07;

    return ESP_OK;
}

esp_err_t tsl2591_set_gain(tsl2591_t *dev, tsl2591_gain_t gain)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_control_register(dev, (dev->settings.control_reg & ~TSL2591_GAIN_MAX) | gain));
    dev->settings.control_reg = (dev->settings.control_reg & ~TSL2591_GAIN_MAX) | gain;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    uint8_t g_idx;
    switch(gain)
    {
      case TSL2591_GAIN_LOW:
        g_idx=0;
      break;
      case TSL2591_GAIN_MEDIUM:
        g_idx=1;
      break;
      case TSL2591_GAIN_HIGH:
        g_idx=2;
      break;
      case TSL2591_GAIN_MAX:
        g_idx=3;
      break;
      default:
        g_idx=0;
      break;
    }

    dev->sen.sen_outs[TSL2591_OUT_CH0_ID].gains_agc.idx=g_idx;
    dev->sen.sen_outs[TSL2591_OUT_CH1_ID].gains_agc.idx=g_idx;

    return ESP_OK;
}

esp_err_t tsl2591_get_gain(tsl2591_t *dev, tsl2591_gain_t *gain)
{
    CHECK_ARG(dev && gain);

    *gain = dev->settings.control_reg & TSL2591_GAIN_MAX;

    return ESP_OK;
}


// Setter and getter persistence filter.
esp_err_t tsl2591_set_persistence_filter(tsl2591_t *dev, tsl2591_persistence_filter_t filter)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_register(dev, TSL2591_REG_PERSIST, (dev->settings.persistence_reg & ~TSL2591_60_CYCLES) | filter));
    dev->settings.persistence_reg = (dev->settings.persistence_reg & ~TSL2591_60_CYCLES) | filter;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_persistence_filter(tsl2591_t *dev, tsl2591_persistence_filter_t *filter)
{
    CHECK_ARG(dev && filter);

    *filter = dev->settings.persistence_reg & TSL2591_60_CYCLES;

    return ESP_OK;
}


// Setters thresholds.
esp_err_t tsl2591_als_set_low_threshold(tsl2591_t *dev, uint16_t low_threshold)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_AILTL, low_threshold));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_AILTH, low_threshold >> 8));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_als_set_high_threshold(tsl2591_t *dev, uint16_t high_threshold)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_AIHTL, high_threshold));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_AIHTH, high_threshold >> 8));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_no_persist_set_low_threshold(tsl2591_t *dev, uint16_t low_threshold)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_NPAILTL, low_threshold));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_NPAILTH, low_threshold >> 8));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_no_persist_set_high_threshold(tsl2591_t *dev, uint16_t high_threshold)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_NPAIHTL, high_threshold));
    I2C_DEV_CHECK(&dev->i2c_dev, write_register(dev, TSL2591_REG_NPAIHTH, high_threshold >> 8));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}


// Special functions.
esp_err_t tsl2591_set_test_intr(tsl2591_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_special_function(dev, TSL2591_SPECIAL_SET_INTR));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_clear_als_intr(tsl2591_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_special_function(dev, TSL2591_SPECIAL_CLEAR_INTR));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_clear_als_np_intr(tsl2591_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_special_function(dev, TSL2591_SPECIAL_CLEAR_NP_INTR));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_clear_both_intr(tsl2591_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        write_special_function(dev, TSL2591_SPECIAL_CLEAR_BOTH));

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}


// Getters status flags.
esp_err_t tsl2591_get_np_intr_flag(tsl2591_t *dev, bool *flag)
{
    CHECK_ARG(dev && flag);

    uint8_t tmp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        read_register(dev, TSL2591_REG_STATUS, &tmp));

    *flag = tmp & TSL2591_STATUS_ALS_NP_INTR ? true : false;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_als_intr_flag(tsl2591_t *dev, bool *flag)
{
    CHECK_ARG(dev && flag);

    uint8_t tmp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        read_register(dev, TSL2591_REG_STATUS, &tmp));

    *flag = tmp & TSL2591_STATUS_ALS_INTR ? true : false;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_get_als_valid_flag(tsl2591_t *dev, bool *flag)
{
    CHECK_ARG(dev && flag);

    uint8_t tmp;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    I2C_DEV_CHECK(&dev->i2c_dev,
        read_register(dev, TSL2591_REG_STATUS, &tmp));

    *flag = tmp & TSL2591_STATUS_ALS_VALID? true : false;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t tsl2591_basic_enable(tsl2591_t *dev)
{
  CHECK_ARG(dev);

  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

  I2C_DEV_CHECK(&dev->i2c_dev,
      write_enable_register(dev, dev->settings.enable_reg | TSL2591_ALS_ON | TSL2591_POWER_ON));
  dev->settings.enable_reg = dev->settings.enable_reg | TSL2591_ALS_ON | TSL2591_POWER_ON;

  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  ESP_LOGD(TAG,"Sleeping for %u ms",int_times[dev->settings.control_reg & 0x07]+10);
  SLEEP_MS(int_times[dev->settings.control_reg & 0x07]+10); //Wait integration time

  return ESP_OK;
}

esp_err_t tsl2591_basic_disable(tsl2591_t *dev)
{
  CHECK_ARG(dev);

  ESP_LOGD(TAG,"Disabeling device and ALS");

  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

  I2C_DEV_CHECK(&dev->i2c_dev,
    write_enable_register(dev, dev->settings.enable_reg & ~TSL2591_ALS_ON & ~TSL2591_POWER_ON));
    dev->settings.enable_reg = dev->settings.enable_reg & ~TSL2591_ALS_ON & ~TSL2591_POWER_ON;

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}
