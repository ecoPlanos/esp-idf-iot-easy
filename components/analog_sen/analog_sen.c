/*
 * Copyright (c) 2019 Ecoplanos: Automação e Sistemas Integrados, Lda.
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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>
#include <string.h>
#include "analog_sen.h"

static const char *TAG = "analog_sen";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

const uint32_t atts[ANALOG_SEN_ATTS_NR] = {ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6, ADC_ATTEN_DB_11};
const uint32_t atts_max_values[ANALOG_SEN_ATTS_NR] = {4095,4095,4095,4095};
const int32_t atts_min_values[ANALOG_SEN_ATTS_NR] = {0,0,0,0};
const float atts_th_h[ANALOG_SEN_ATTS_NR] = {0.85,0.85,0.85,0.85};
const float atts_th_l[ANALOG_SEN_ATTS_NR] = {0.20,0.20,0.20,0.20};

static void check_efuse(void){
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: Supported\n");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(TAG, "Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This is configured for ESP32/ESP32S2."
#endif
}

static void print_char_val_type(esp_adc_cal_value_t val_type){
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    ESP_LOGI(TAG,"Characterized using Two Point Value\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    ESP_LOGI(TAG,"Characterized using eFuse Vref\n");
  } else {
    ESP_LOGI(TAG,"Characterized using Default Vref\n");
  }
}

static inline analog_sen_att_t idx2att(uint8_t idx) {
  switch(idx) {
    case 0:
      return ANALOG_SEN_ATT_0DB;
    break;
    case 1:
      return ANALOG_SEN_ATT_2_5DB;
    break;
    case 2:
      return ANALOG_SEN_ATT_6DB;
    break;
    case 3:
      return ANALOG_SEN_ATT_11DB;
    break;
    default:
      return ANALOG_SEN_ATT_0DB;
    break;
  }
}

static inline esp_err_t analog_sen_test_att(analog_sen_t *dev) {
  sen_agc_change_type_t agc_change = 0;
  analog_sen_att_t att;
  uint8_t i;
  for(i=0;i<dev->outs_nr;i++){
    if (dev->outs[i].adc_unit == ADC_UNIT_1) {
      agc_change = sensor_out_agc_change(dev->sen.outs[i], (uint32_t)adc1_get_raw((adc1_channel_t)dev->sen.outs[i].gpio));
    } else {
      int raw;
      adc2_get_raw((adc2_channel_t)dev->sen.outs[i].gpio, dev->sen.outs[i].gpio, &raw);
      agc_change = sensor_out_agc_change(dev->sen.outs[i], raw);
    }
  }
  // uint16_t adc_reading = 0;
  // while(agc_change !=  SEN_AGC_CHANGE_NOP) {
  if(agc_change !=  SEN_AGC_CHANGE_NOP) {
    ESP_LOGI(TAG,"agc_change: %u",agc_change);
    if(agc_change==SEN_AGC_CHANGE_UP)
    {
      if(dev->sen.outs[0].atts_agc.idx < dev->sen.outs[0].atts_agc.val_nr-1)
      {
        ESP_LOGI(TAG,"Attenuation too low. Adjusting att UP...");
        CHECK(analog_sen_set_att(dev,dev->sen.outs[0].atts_agc.idx+1));
        return ESP_ERR_INVALID_STATE;
      }
      else {
        ESP_LOGW(TAG,"Sensor saturated!");
        ESP_LOGD(TAG,"dev->sen.outs[0].atts_agc.idx: %u",dev->sen.outs[0].atts_agc.idx);
        // gettimeofday(&tv, NULL);
        // dev->sen.timestamp = tv.tv_sec * 1000000LL + tv.tv_usec;
        // dev->sen.esp_timestamp = esp_timer_get_time();
        // dev->sen.outs[0].m_raw = adc1_get_raw((adc1_channel_t)dev->sen.outs[0].gpio);
        //TODO: add flag to data to indicate saturation!
        // return ESP_OK;
      }
    } else if(agc_change==SEN_AGC_CHANGE_DOWN) {
      if(dev->sen.outs[0].atts_agc.idx > 0)
      {
        ESP_LOGI(TAG,"Attenuation too high. Adjusting att DOWN...");
        CHECK(analog_sen_set_att(dev,dev->sen.outs[0].atts_agc.idx-1));
        return ESP_ERR_INVALID_STATE;
      }
      else
      {
        ESP_LOGI(TAG,"Reached minimum attenuation!");
        // break;
      }
    }
    // if (dev->adc_unit == ADC_UNIT_1) {
    //     agc_change = sensor_out_agc_change(dev->sen.outs[0], (uint32_t)adc1_get_raw((adc1_channel_t)dev->sen.outs[0].gpio));
    // } else {
    //     int raw;
    //     adc2_get_raw((adc2_channel_t)dev->sen.outs[0].gpio, ADC_WIDTH_BIT_12, &raw);
    //     agc_change = sensor_out_agc_change(dev->sen.outs[0], raw);
    // }
  }
  return ESP_OK;
}

static inline esp_err_t analog_sen_outs_init(analog_sen_t *dev) {
  uint8_t i,j;
  if(dev->outs_nr < 1){
    ESP_LOGE(TAG, "call analog_sen_init_desc() with at least 1 output.");
    return ESP_ERR_INVALID_ARG;
  }
  dev->outs = (analog_out_t*) malloc(dev->outs_nr*sizeof(analog_out_t));
  for(i=0;i<dev->outs_nr;i++) {
    dev->outs[i].adc_mean = 0;
    // dev->outs[i].adc_chars = malloc(sizeof(esp_adc_cal_characteristics_t));
    // dev->outs[i].adc_chars = calloc(4, sizeof(esp_adc_cal_characteristics_t));
    // dev->outs[i].adc_chars = (esp_adc_cal_characteristics_t*) malloc(4*sizeof(esp_adc_cal_characteristics_t));
    dev->outs[i].configured = 0;
    // for(j=0;j<4;j++){
      // if(dev->outs[i].adc_chars[j]) memset(dev->outs[i].adc_chars[j],0, sizeof(esp_adc_cal_characteristics_t));
      // else ESP_LOGE(TAG, "adc_chars not set!");
      // dev->outs[i].adc_chars[j] = (esp_adc_cal_characteristics_t*)malloc(sizeof(esp_adc_cal_characteristics_t));
      // memset(dev->outs[i].adc_chars[j],0, sizeof(esp_adc_cal_characteristics_t));
      // dev->outs[i].adc_chars[j] =(esp_adc_cal_characteristics_t*) malloc(sizeof(esp_adc_cal_characteristics_t)));
    // }
  }
  return ESP_OK;
}

// Initialization.
esp_err_t analog_sen_init_desc( analog_sen_t *dev, \
                                uint8_t samples_filter, uint32_t period_ms, \
                                uint16_t sen_id, char sen_name[], \
                                uint8_t outs_nr, void *calc_processed) {
    uint8_t i;
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Initialize descriptor");
    dev->outs_nr = outs_nr;
    dev->calc_processed = calc_processed;
    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,outs_nr);
    strcpy(dev->sen.info.name, sen_name);
    dev->sen.info.lib_id = SEN_ANALOG_SEN_LIB_ID;
    dev->sen.info.sen_id = sen_id;
    dev->sen.info.version = 1;
    dev->sen.info.com_type = SEN_COM_TYPE_ANALOG;
    dev->sen.conf.samples_filter = samples_filter;
    // dev->sen.conf.period_ms=nearest_prime(CONFIG_ANALOG_DEFAULT_PERIOD_MS);
    dev->sen.conf.period_ms=period_ms;
    dev->sen.conf.min_period_us = samples_filter*5000;
    dev->sen.conf.delay_start_get_us = 10000;
    dev->sen.conf.delay_after_awake_us = 10000;
    dev->sen.conf.time_to_adjust_us = 10000*(ANALOG_SEN_ATTS_NR+1);
    dev->sen.info.out_nr = outs_nr;
    dev->sen.info.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.esp_timestamp=0;
    dev->sen.dev=dev;
    dev->sen.reset=analog_sen_iot_sen_reset;
    dev->sen.reinit=analog_sen_iot_sen_reinit;
    dev->sen.start_measurement=analog_sen_iot_sen_start_measurement;
    dev->sen.get_data=analog_sen_iot_sen_get_data;
    dev->sen.awake=analog_sen_iot_sen_sleep_mode_awake;
    // dev->sen.gain_adjust=analog_sen_iot_sen_gain_adjust;
    dev->sen.gain_adjust=analog_sen_iot_sen_start_measurement;
    dev->sen.sleep=analog_sen_iot_sen_sleep_mode_sleep;

    dev->sen.status.fail_cnt = 0;
    dev->sen.status.fail_time = 0;
    CHECK(analog_sen_outs_init(dev));
    dev->sen.conf.srate=0;
    // dev->sen.outs[0].timestamp=0;
    for(i=0; i<outs_nr; i++){
      CHECK(sensor_out_agc_init(&dev->sen.outs[i].atts_agc, SEN_AGC_TYPE_ATT, ANALOG_SEN_ATTS_NR, \
        atts, atts_max_values, atts_min_values, atts_th_h, atts_th_l));
      dev->sen.outs[i].itimes_agc.state = 0;
      dev->sen.outs[i].atts_agc.idx=CONFIG_ANALOG_SEN_DEFAULT_ATT;
    }
    // dev->sen.outs[0].gains_agc.state = false;
    return ESP_OK;
}

esp_err_t analog_sen_free_desc(analog_sen_t *dev) {
    CHECK_ARG(dev);
    ESP_LOGD(TAG, "Free descriptor.");

    return ESP_OK;
}

esp_err_t analog_sen_config_output(analog_sen_t *dev, uint8_t out_idx, adc_unit_t unit, adc_channel_t analog_channel, void *calc_processed) {
  dev->sen.outs[out_idx].out_id=out_idx;
  dev->sen.outs[out_idx].out_val_type = SEN_OUT_VAL_TYPE_UINT16;
  dev->sen.outs[out_idx].gpio=analog_channel;
  dev->sen.outs[out_idx].bit_nr=16;
  dev->sen.outs[out_idx].m_raw=0;
  esp_adc_cal_characterize(unit, ANALOG_SEN_ATT_0DB, ADC_WIDTH_BIT_12, ANALOG_SEN_DEFAULT_VREF, &dev->outs[out_idx].adc_chars[ANALOG_SEN_ATT_0DB]);
  esp_adc_cal_characterize(unit, ANALOG_SEN_ATT_2_5DB, ADC_WIDTH_BIT_12, ANALOG_SEN_DEFAULT_VREF, &dev->outs[out_idx].adc_chars[ANALOG_SEN_ATT_2_5DB]);
  esp_adc_cal_characterize(unit, ANALOG_SEN_ATT_6DB, ADC_WIDTH_BIT_12, ANALOG_SEN_DEFAULT_VREF, &dev->outs[out_idx].adc_chars[ANALOG_SEN_ATT_6DB]);
  esp_adc_cal_characterize(unit, ANALOG_SEN_ATT_11DB, ADC_WIDTH_BIT_12, ANALOG_SEN_DEFAULT_VREF, &dev->outs[out_idx].adc_chars[ANALOG_SEN_ATT_11DB]);
  dev->outs[out_idx].adc_unit=unit;
  dev->outs[out_idx].analog_channel=analog_channel;
  dev->outs[out_idx].calc_processed=calc_processed;
  dev->outs[out_idx].configured=1;
  return ESP_OK;
}

esp_err_t analog_sen_init(analog_sen_t *dev) {
  CHECK_ARG(dev);
  uint8_t i;
  ESP_LOGD(TAG, "Initialize sensor.");

  check_efuse();
  adc1_config_width(ADC_WIDTH_BIT_12);
  for(i=0; i<dev->outs_nr; i++){
    adc1_config_channel_atten(dev->sen.outs[i].gpio, dev->sen.outs[i].atts_agc.idx);
  }

  dev->sen.status.status_code = SEN_STATUS_OK;

  return ESP_OK;
}
// Measure.
esp_err_t analog_sen_get_channel_data(analog_sen_t *dev) {
    // uint64_t timestamp;
    CHECK_ARG(dev);

    uint8_t i,j;
    sen_agc_change_type_t agc_change;
    uint32_t adc_reading_mean = 0;
    uint16_t adc_val;
    uint8_t zero_count=0;
    ESP_LOGD(TAG, "analog_sen_get_channel_data");
    // if (dev->adc_unit == ADC_UNIT_1) {
    //     agc_change = sensor_out_agc_change(dev->sen.outs[0], (uint32_t)adc1_get_raw((adc1_channel_t)dev->sen.outs[0].gpio));
    // } else {
    //     int raw;
    //     adc2_get_raw((adc2_channel_t)dev->sen.outs[0].gpio, dev->sen.outs[0].gpio, &raw);
    //     agc_change = sensor_out_agc_change(dev->sen.outs[0], raw);
    // }
    // uint16_t adc_reading = 0;
    // while(agc_change !=  SEN_AGC_CHANGE_NOP) {
    // if(analog_sen_test_att(dev)!=ESP_OK)
    //   return ESP_FAIL;
    //Multisampling
    int64_t m_mon = esp_timer_get_time();
    for(j=0; j<dev->outs_nr; j++) dev->outs[j].adc_mean=0;
    for (i = 0; i < dev->sen.conf.samples_filter; i++) {
      // zero_count=0;
      for(uint8_t j=0; j<dev->outs_nr; j++) {
        if (dev->outs[j].adc_unit == ADC_UNIT_1) {
          adc_val = adc1_get_raw((adc1_channel_t)dev->sen.outs[j].gpio);
          // if(adc_val > 0)
            dev->outs[j].adc_mean += adc_val;
          // else{
          //   i--;
          //   zero_count++;
          //   if(zero_count == dev->sen.conf.samples_filter) {
          //     dev->outs[j].adc_mean = 0;
          //     break;
          //   }
          // }
        } else {
          int raw;
          adc2_get_raw((adc2_channel_t)dev->sen.outs[j].gpio, ADC_WIDTH_BIT_12, &raw);
          dev->outs[j].adc_mean += raw;
        }
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    for(j=0; j<dev->outs_nr; j++){
      dev->outs[j].adc_mean /= dev->sen.conf.samples_filter;
      dev->sen.outs[j].m_raw = dev->outs[j].adc_mean;
      ESP_LOGD(TAG,"adc_reading_mean %u: %u", j,dev->outs[j].adc_mean);
    }
    dev->sen.esp_timestamp = esp_timer_get_time();
    dev->sen.esp_timestamp = dev->sen.esp_timestamp-((dev->sen.esp_timestamp - m_mon)/2);
    return ESP_OK;
}

esp_err_t analog_sen_get_voltage(analog_sen_t *dev, uint16_t adc_reading, uint8_t out_idx, uint16_t *voltage) {
    CHECK_ARG(dev);
    //TODO: check voltage dividers
    *voltage = esp_adc_cal_raw_to_voltage(adc_reading, &dev->outs[out_idx].adc_chars[dev->sen.outs[out_idx].atts_agc.idx]);
    dev->sen.outs[out_idx].voltage=(float)*voltage;
    dev->outs[out_idx].voltage=*voltage;
    ESP_LOGD(TAG, "Voltage out[%u]: %u mV",out_idx,*voltage);
    return ESP_OK;
}

esp_err_t analog_sen_set_att(analog_sen_t *dev, analog_sen_att_t att) {
  CHECK_ARG(dev);
  uint8_t i;
  for(i=0;i<dev->outs_nr;i++){
    if (dev->outs[i].adc_unit == ADC_UNIT_1) {
      CHECK(adc1_config_channel_atten(dev->sen.outs[i].gpio, att));
    } else {
      CHECK(adc2_config_channel_atten((adc2_channel_t)dev->sen.outs[i].gpio, att));
    }
    dev->sen.outs[i].atts_agc.idx=att;
  }

  return ESP_OK;
}

esp_err_t analog_sen_get_att(analog_sen_t *dev, analog_sen_att_t *att) {
    CHECK_ARG(dev && att);


    return ESP_OK;
}

// IoT Dev specific functions
esp_err_t analog_sen_iot_sen_start_measurement(void *dev) {
  ESP_LOGD(TAG,"IOT start measurement");
  return analog_sen_test_att((analog_sen_t*) dev);
}

esp_err_t analog_sen_iot_sen_get_data(void *dev) {
  analog_sen_t *analog_dev = (analog_sen_t *)dev;
  uint16_t voltage;
  uint8_t i;
  ESP_LOGD(TAG, "analog_sen_iot_sen_get_data");
  analog_sen_get_channel_data(analog_dev);
  for(i=0;i<analog_dev->outs_nr;i++){
    ESP_LOGD(TAG, "raw[%u]: %u",i, analog_dev->sen.outs[i].m_raw);
    analog_sen_get_voltage(analog_dev, analog_dev->sen.outs[i].m_raw, i, &voltage);
    ESP_LOGD(TAG, "voltage[%u]: %u mv",i, voltage);
    analog_dev->sen.outs[i].m_raw=voltage;
    if(analog_dev->outs[i].calc_processed != NULL)
      analog_dev->outs[i].calc_processed(analog_dev);
  }
  if(analog_dev->calc_processed != NULL)
    analog_dev->calc_processed(analog_dev);

  return ESP_OK;
}

esp_err_t analog_sen_iot_sen_sleep_mode_awake(void *dev) {
  // analog_sen_t* dev_ = (analog_sen_t*) dev;
  ESP_LOGD(TAG,"IOT awake");
  return ESP_OK;
}

esp_err_t analog_sen_iot_sen_sleep_mode_sleep(void *dev) {
  // analog_sen_t* dev_ = (analog_sen_t*) dev;
  ESP_LOGD(TAG,"IOT sleep");
  return ESP_OK;
}

esp_err_t analog_sen_iot_sen_reset(void *dev) {
  ESP_LOGD(TAG,"IOT reset");
  return ESP_OK;
}

esp_err_t analog_sen_iot_sen_reinit(void *dev) {
  ESP_LOGD(TAG,"IOT reinit");
  return ESP_OK;
}
