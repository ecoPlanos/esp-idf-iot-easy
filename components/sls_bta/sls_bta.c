/*
 * Copyright (c) 2022 Ecoplanos: Automação e Sistemas Integrados, Lda.
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
#include <esp_timer.h>
#include <string.h>
#include "sls_bta.h"

static const char *TAG = "SLS_BTA";

#define K0 30.3014
#define K1 13.8580
#define K2  2.1730

// static void sls_bta_calc_sound(sensor_t *sen) {
//
//   // ESP_LOGD(TAG, "sen->outs[0].sound_pressure: %f",sen->outs[0].sound_pressure);
//   // ESP_LOGD(TAG, "K0: %f",K0);
//   // ESP_LOGD(TAG, "K1: %f",K1);
//   // ESP_LOGD(TAG, "K2: %f",K2);
// }

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SLEEP_MS(x) do { vTaskDelay(pdMS_TO_TICKS(x)); } while (0)

// static inline esp_err_t analog_sen_outs_init(analog_sen_t *dev) {
//   uint8_t i;
//   if(dev->outs_nr < 1){
//     ESP_LOGE(TAG, "call analog_sen_init_desc() with at least 1 output.");
//     return ESP_ERR_INVALID_ARG;
//   }
//   dev->outs = malloc(dev->outs_nr*sizeof(analog_out_t));
//   for(i=0;i<dev->outs_nr;i++) {
//     dev->outs[i].adc_chars = 0;
//     dev->outs[i].adc_mean = 0;
//     // dev->outs[i].adc_chars = malloc(sizeof(esp_adc_cal_characteristics_t));
//     dev->outs[i].adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//     dev->outs[i].configured = 0;
//   }
//   return ESP_OK;
// }

esp_err_t sls_bta_init(analog_sen_t *sls_bta_sen, uint8_t samples_filter, uint32_t period_ms, uint16_t sen_id, adc_unit_t sound_unit, adc_channel_t sound_channel, adc_unit_t ref_unit, adc_channel_t ref_channel, char *sen_name){
  CHECK_ARG(sls_bta_sen);
  CHECK(analog_sen_init_desc(sls_bta_sen, samples_filter, period_ms, sen_id, sen_name, 2, sls_bta_calc_sound_pressure));
  CHECK(analog_sen_config_output(sls_bta_sen, SLS_BTA_OUT_SOUND_ID, sound_unit, sound_channel, NULL));
  CHECK(analog_sen_config_output(sls_bta_sen, SLS_BTA_OUT_REF_ID, ref_unit, ref_channel, NULL));

  sls_bta_sen->sen.outs[SLS_BTA_OUT_SOUND_ID].out_type = SEN_TYPE_SOUND_PRESSURE;
  sls_bta_sen->sen.outs[SLS_BTA_OUT_SOUND_ID].sound_pressure=0.0;
  sls_bta_sen->sen.outs[SLS_BTA_OUT_REF_ID].out_type = SEN_TYPE_VOLTAGE;
  sls_bta_sen->sen.outs[SLS_BTA_OUT_REF_ID].voltage=0.0;
  CHECK(analog_sen_init(sls_bta_sen));

  return ESP_OK;
}

esp_err_t sls_bta_calc_sound_pressure(void *sls_bta_sen){
  analog_sen_t *sls_bta_sen_ = (analog_sen_t *)sls_bta_sen;
  ESP_LOGD(TAG, "sound voltage: %f mv", sls_bta_sen_->sen.outs[SLS_BTA_OUT_SOUND_ID].voltage);
  ESP_LOGD(TAG, "ref voltage: %f mv", sls_bta_sen_->sen.outs[SLS_BTA_OUT_REF_ID].voltage);
  sls_bta_sen_->sen.outs[SLS_BTA_OUT_SOUND_ID].sound_pressure = K0+K1*(sls_bta_sen_->sen.outs[SLS_BTA_OUT_SOUND_ID].voltage/655.963302752) +
                                K2*(sls_bta_sen_->sen.outs[SLS_BTA_OUT_SOUND_ID].voltage/655.963302752)*(sls_bta_sen_->sen.outs[SLS_BTA_OUT_SOUND_ID].voltage/655.963302752);
  sls_bta_sen_->sen.outs[SLS_BTA_OUT_REF_ID].voltage = sls_bta_sen_->sen.outs[SLS_BTA_OUT_REF_ID].voltage/655.963302752;
  return ESP_OK;
}
