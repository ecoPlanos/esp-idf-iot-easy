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


#ifndef __ANALOG_SEN_H__
#define __ANALOG_SEN_H__

#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <sensor_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ANALOG_SEN_LIB_VERSION_MAJOR 2
#define ANALOG_SEN_LIB_VERSION_MINOR 0
#define ANALOG_SEN_LIB_VERSION_PATCH 0
#define ANALOG_SEN_LIB_VERSION  (ANALOG_SEN_LIB_VERSION_MAJOR << 16)|(ANALOG_SEN_LIB_VERSION_MINOR << 8)|ANALOG_SEN_LIB_VERSION_PATCH

#define ANALOG_SEN_ATTS_NR 4
#define ANALOG_SEN_DEFAULT_VREF    1100

/**
 * Attenuation.
 */
typedef enum
{
    ANALOG_SEN_ATT_0DB = 0,
    ANALOG_SEN_ATT_2_5DB,
    ANALOG_SEN_ATT_6DB,
    ANALOG_SEN_ATT_11DB
} analog_sen_att_t;

/**
 * Device info.
 */
typedef struct
{
  uint8_t pack_id;    // Package Identification
  uint8_t dev_id;     // Device Identification
  uint8_t status;
} analog_sen_inf_t;

/**
 * Analog output descriptor.
 */
typedef struct
{
  esp_adc_cal_characteristics_t adc_chars[ANALOG_SEN_ATTS_NR];
  adc_unit_t adc_unit;
  adc_channel_t analog_channel;
  uint32_t adc_mean;
  uint16_t voltage;
  uint8_t configured;
  void (*calc_processed)(void *sen);
} analog_out_t;

/**
 * Device descriptor.
 */
typedef struct
{
  uint8_t outs_nr;
  analog_out_t *outs;
  analog_sen_inf_t info;
  void (*calc_processed)(void *sen);
  sensor_t sen;
} analog_sen_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param analog_channel Analog GPIO pin
 * @param sen_id Sensor Id
 * @param sen_name Sensor Name
 * @param outs_nr Number of sensor analog outputs
 * @param period_ms Sensor period in milliseconds
 * @return `ESP_OK` on success
 */
esp_err_t analog_sen_init_desc( analog_sen_t *dev, \
                                uint8_t samples_filter, uint32_t period_ms, \
                                uint16_t sen_id, char sen_name[], \
                                uint8_t outs_nr, \
                                void *calc_processed_func);

/**
* @brief Free device descriptor
*
* @param dev Device descriptor
* @param out_idx Sensor output index (id)
* @param unit Output ADC unit
* @param analog_channel Output ADC channel
* @return `ESP_OK` on success
*/
esp_err_t analog_sen_config_output(analog_sen_t *dev, uint8_t out_idx, adc_unit_t unit, adc_channel_t analog_channel, void *calc_processed);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t analog_sen_free_desc(analog_sen_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t analog_sen_init(analog_sen_t *dev);

/**
 * @brief Read channel data and store it on the sensor handler structure
 *
 * @param dev Device descriptor
 */
esp_err_t analog_sen_iot_sen_measurement(void *dev);

/**
 * @brief Read channel data
 *
 * @param dev Device descriptor
 */
esp_err_t analog_sen_get_channel_data(analog_sen_t *dev);

/**
 * @brief Calculate voltage
 *
 * @param dev Device descriptor
 * @param channel0 Channel0 data
 * @param channel1 Channel1 data
 * @param[out] lux Light intensity
 * @return `ESP_OK` on success
 */
esp_err_t analog_sen_get_voltage(analog_sen_t *dev, uint16_t adc_reading, uint8_t out_idx, uint16_t *voltage);

/**
 * @brief Set device att
 *
 * @param dev Device descriptor
 * @param att Attenuation
 * @return `ESP_OK` on success
 */
esp_err_t analog_sen_set_att(analog_sen_t *dev, analog_sen_att_t att);

/**
 * @brief Get device att
 *
 * @param dev Device descriptor
 * @param[out] att Attenuation
 * @return `ESP_OK` on success
 */
esp_err_t analog_sen_get_att(analog_sen_t *dev, analog_sen_att_t *att);


/**************************************************************************/
/*!
    @brief  Enables the chip, so it's ready to take readings
*/
/**************************************************************************/
esp_err_t analog_sen_basic_enable(analog_sen_t *dev);

/**************************************************************************/
/*!
    @brief Disables the chip, so it's in power down mode
*/
/**************************************************************************/
esp_err_t analog_sen_basic_disable(analog_sen_t *dev);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t analog_sen_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t analog_sen_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t analog_sen_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t analog_sen_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t analog_sen_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t analog_sen_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __ANALOG_SEN_H__
