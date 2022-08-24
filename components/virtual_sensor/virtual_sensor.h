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
 * @file virtual_sensor.h
 * @defgroup virtual_sensor virtual_sensor
 * @{
 *
 * ESP-IDF driver for Sensirion VIRTUAL_SENSOR digital temperature and humidity sensor
 *
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __VIRTUAL_SENSOR_H__
#define __VIRTUAL_SENSOR_H__

#include <stdbool.h>
#include <esp_err.h>
#include <sensor_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VIRTUAL_SENSOR_LIB_VERSION_MAJOR 1
#define VIRTUAL_SENSOR_LIB_VERSION_MINOR 0
#define VIRTUAL_SENSOR_LIB_VERSION_PATCH 0
#define VIRTUAL_SENSOR_LIB_VERSION  (VIRTUAL_SENSOR_LIB_VERSION_MAJOR << 16)|(VIRTUAL_SENSOR_LIB_VERSION_MINOR << 8)|VIRTUAL_SENSOR_LIB_VERSION_PATCH

#define VIRTUAL_SENSOR_RAW_DATA_SIZE 6
// #define VIRTUAL_SENSOR_RAW_DATA_SIZE 6*2

enum
{
  VIRTUAL_SENSOR_OUT_AWAKE_TIME_ID = 0,
  VIRTUAL_SENSOR_OUT_MEAS_START_TIME_ID,
  VIRTUAL_SENSOR_OUT_MEAS_GET_TIME_ID,
  VIRTUAL_SENSOR_OUT_RESET_COUNTER_ID
};

typedef enum {
  VIRTUAL_SENSOR_SLEEP_MODE_SLEEP,
  VIRTUAL_SENSOR_SLEEP_MODE_AWAKE
} virtual_sensor_sleep_modes;

typedef uint8_t virtual_sensor_raw_data_t[VIRTUAL_SENSOR_RAW_DATA_SIZE];

/**
 * Device descriptor
 */
typedef struct
{
  bool sleep_mode;
  bool meas_started;            //!< indicates whether measurement started
  uint64_t meas_awake_time;
  uint64_t meas_start_time;     //!< measurement start time in us
  uint64_t meas_get_time;
  uint32_t reset_count;
  sensor_t sen;
} virtual_sensor_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param addr      VIRTUAL_SENSOR address
 * @param scl_gpio  SCL GPIO
 * @param sen_id    Sensor ID
 * @return          ESP_OK` on success
 */
esp_err_t virtual_sensor_init_desc(virtual_sensor_t *dev, uint16_t sen_id);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t virtual_sensor_free_desc(virtual_sensor_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t virtual_sensor_init(virtual_sensor_t *dev);

/**
* @brief Sensor to sleep mode
*
* @param dev       Device descriptor
* @return          `ESP_OK` on success
*/
esp_err_t virtual_sensor_sleep(virtual_sensor_t *dev);

/**
 * @brief Sensor awake from sleep mode
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t virtual_sensor_awake(virtual_sensor_t *dev);

/**
 * @brief Reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t virtual_sensor_reset(virtual_sensor_t *dev);

/**
 * @brief Read sensor data and store it on the sensor handler structure
 *
 * @param dev Device descriptor
 */
esp_err_t virtual_sensor_iot_sen_measurement(void *dev);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t virtual_sensor_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t virtual_sensor_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t virtual_sensor_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t virtual_sensor_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t virtual_sensor_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t virtual_sensor_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __VIRTUAL_SENSOR_H__ */
