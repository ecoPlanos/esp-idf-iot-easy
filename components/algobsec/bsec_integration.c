/*
 * Copyright (c) 2022 ecoPlanos <geral@ecoplanos.pt>
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
 * @file bsec_integration.c
 *
 * ESP-IDF driver for Sensirion PMS1003 digital temperature and humidity sensor
 *
 * Copyright (c) 2022 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include "bsec_interface.h"
//
// uint8_t bme680_iot_sen_get_data(void *dev) {
//   bme680_t *dev_ = (bme680_t *) dev;
//   uint8_t n_outputs;
//   bsec_library_return_t status;
//   bsec_input_t inputs[4];
//   bsec_output_t outputs[14];
//   bme680_values_float_t bme680_values_float;
//   CHECK(bme680_get_results_float((bme680_t*) dev, &bme680_values_float));
//   inputs[0].time_stamp = dev_->sen.timestamp;
//   inputs[1].time_stamp = dev_->sen.timestamp;
//   inputs[2].time_stamp = dev_->sen.timestamp;
//   inputs[3].time_stamp = dev_->sen.timestamp;
//   inputs[0].signal = bme680_values_float.temperature;
//   inputs[1].signal = bme680_values_float.pressure;
//   inputs[2].signal = bme680_values_float.humidity;
//   inputs[3].signal = bme680_values_float.gas_resistance;
//   inputs[0].signal_dimensions = 1;
//   inputs[1].signal_dimensions = 1;
//   inputs[2].signal_dimensions = 1;
//   inputs[3].signal_dimensions = 1;
//   inputs[0].sensor_id = BSEC_INPUT_TEMPERATURE;
//   inputs[1].sensor_id = BSEC_INPUT_PRESSURE;
//   inputs[2].sensor_id = BSEC_INPUT_HUMIDITY;
//   inputs[3].sensor_id = BSEC_INPUT_GASRESISTOR;
//   status=BSEC_E_DOSTEPS_INVALIDINPUT;
//   status = bsec_do_steps(inputs, 4, outputs, &n_outputs);
//   if(status == BSEC_OK) {
//     for(int i = 0; i < 4; i++) {
//       ESP_LOGD(TAG, "BSEC out[%u] returned sensor id: %d",i,outputs[i].sensor_id);
//       switch(outputs[i].sensor_id) {
//         case BSEC_OUTPUT_IAQ:
//           // Retrieve the IAQ results from output[i].signal
//           // and do something with the data
//         break;
//         case BSEC_OUTPUT_STATIC_IAQ:
//         break;
//         case BSEC_OUTPUT_CO2_EQUIVALENT:
//         break;
//         case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
//         break;
//         case BSEC_OUTPUT_RAW_TEMPERATURE:
//         break;
//         case BSEC_OUTPUT_RAW_PRESSURE:
//         break;
//         case BSEC_OUTPUT_RAW_HUMIDITY:
//         break;
//         case BSEC_OUTPUT_RAW_GAS:
//         break;
//         case BSEC_OUTPUT_STABILIZATION_STATUS:
//         break;
//         case BSEC_OUTPUT_RUN_IN_STATUS:
//         break;
//         case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
//         break;
//         case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
//         break;
//         case BSEC_OUTPUT_COMPENSATED_GAS:
//         break;
//         case BSEC_OUTPUT_GAS_PERCENTAGE:
//         break;
//       }
//     }
//   }
//   return ESP_OK;
// }
//
// esp_err_t bme680_iot_sen_sleep_mode_awake(void *dev) {
//   bme680_t* dev_ = (bme680_t*) dev;
//   return ESP_OK;
//   // if((dev_->settings.enable_reg & TSL2591_ALS_ON) && (dev_->settings.enable_reg & TSL2591_POWER_ON))
//   //   return bme680_basic_enable(dev_);
//   // else
//   //   return bme680_basic_disable(dev_);
// }
//
// esp_err_t bme680_iot_sen_sleep_mode_sleep(void *dev) {
//   bme680_t* dev_ = (bme680_t*) dev;
//   return ESP_OK;
//   // if((dev_->settings.enable_reg & TSL2591_ALS_ON) && (dev_->settings.enable_reg & TSL2591_POWER_ON))
//   //   return bme680_basic_enable(dev_);
//   // else
//   //   return bme680_basic_disable(dev_);
// }
//
// esp_err_t bme680_iot_sen_reset(void *dev) {
//   return ESP_OK;
// }
//
// esp_err_t bme680_iot_sen_reinit(void *dev) {
//   return ESP_OK;
// }
