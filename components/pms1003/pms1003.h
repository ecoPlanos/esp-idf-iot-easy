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
 * @file pms1003.h
 * @defgroup pms1003 pms1003
 * @{
 *
 * ESP-IDF driver for Sensirion PMS1003 digital temperature and humidity sensor
 *
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __PMS1003_H__
#define __PMS1003_H__

#include <stdbool.h>
#include <esp_err.h>
#include <sensor_handler.h>
#include <uartdev.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PMS1003_LIB_VERSION_MAJOR 1
#define PMS1003_LIB_VERSION_MINOR 0
#define PMS1003_LIB_VERSION_PATCH 0
#define PMS1003_LIB_VERSION  (PMS1003_LIB_VERSION_MAJOR << 16)|(PMS1003_LIB_VERSION_MINOR << 8)|PMS1003_LIB_VERSION_PATCH

#define LIB_VERSION_PMS1003 "V1.0"

#define PMS1003_RAW_DATA_SIZE 28
#define PMS1003_TOTAL_DATA_SIZE 32

#define PMS1003_BAUD_RATE_DEFAULT 9600  // Default serial setup: 9600bps Check bit：None Stop bit：1 bit

typedef enum {
  PMS1003_OUT_PM1_0_CON_UNIT_ID = 0,  //DATA1
  PMS1003_OUT_PM2_5_CON_UNIT_ID,  //DATA2
  PMS1003_OUT_PM10_CON_UNIT_ID,  //DATA3
  PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID,  //DATA4
  PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID,  //DATA5
  PMS1003_OUT_CON_UNIT_ATMOSPHE_ID,  //DATA6
  PMS1003_OUT_PARTICLE_NR_0_3_UM_ID,  //DATA7
  PMS1003_OUT_PARTICLE_NR_0_5_UM_ID,  //DATA8
  PMS1003_OUT_PARTICLE_NR_1_0_UM_ID,  //DATA9
  PMS1003_OUT_PARTICLE_NR_2_5_UM_ID,  //DATA10
  PMS1003_OUT_PARTICLE_NR_5_0_UM_ID,  //DATA11
  PMS1003_OUT_PARTICLE_NR_10_UM_ID  //DATA12
}pms1003_out_ids_t;

// typedef uint8_t pms1003_raw_data_t[PMS1003_RAW_DATA_SIZE];

typedef struct {
  uint16_t pm1_0_con_unit;  //DATA1
  uint16_t pm2_5_con_unit;  //DATA2
  uint16_t pm10_con_unit;  //DATA3
  uint16_t pm1_0_con_unit_atmosphe;  //DATA4
  uint16_t pm2_5_con_unit_atmosphe;  //DATA5
  uint16_t con_unit_atmosphe;  //DATA6
  uint16_t particle_nr_0_3_um;  //DATA7
  uint16_t particle_nr_0_5_um;  //DATA8
  uint16_t particle_nr_1_0_um;  //DATA9
  uint16_t particle_nr_2_5_um;  //DATA10
  uint16_t particle_nr_5_0_um;  //DATA11
  uint16_t particle_nr_10_um;  //DATA12
  uint16_t reserved;  //DATA13
  uint16_t check;  //Check code = Start character 1 + Start character 2 + ... + data 13 Low 8 bits
} pms1003_raw_data_t;

/**
 * Possible acquisition modes
 */
typedef enum {
  PMS1003_DATA_MODE_PASSIVE = 0,    /**Passive mode */
  PMS1003_DATA_MODE_ACTIVE = 1       /**Active mode */
}pms1003_mode_type_t;

typedef enum {
  PMS1003_SLEEP_MODE_SLEEP = 0,     /**Passive mode */
  PMS1003_SLEEP_MODE_AWAKE = 1       /**Active mode */
}pms1003_sleep_type_t;

typedef struct {
  pms1003_mode_type_t mode;
  pms1003_sleep_type_t sleep_mode;
} pms1003_status_t;

/**
 * Device info.
 */
typedef struct {
  uint8_t pack_id;    // Package Identification
  uint8_t dev_id;     // Device Identification
} pms1003_inf_t;

/**
 * Device config.
 */
typedef struct {
  uint32_t delay_after_awake_ms;    // delay before measuring after a wake from sleep event
} pms1003_conf_t;

/**
 * Device descriptor
 */
typedef struct {
  uart_dev_t uart_dev;
  uint32_t serial;              //!< device serial number

  bool meas_started;            //!< indicates whether measurement started
  uint64_t meas_start_time;     //!< measurement start time in us
  pms1003_inf_t info;
  pms1003_status_t status;
  pms1003_conf_t conf;
  sensor_t sen;
} pms1003_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param addr      PMS1003 address
 * @param port      UART port
 * @param tx_gpio   TX GPIO
 * @param rx_gpio   RX GPIO
 * @return          ESP_OK` on success
 */
esp_err_t pms1003_init_desc(pms1003_t *dev, uart_port_t port, gpio_num_t tx_gpio, gpio_num_t rx_gpio, gpio_num_t rts_io_num, gpio_num_t cts_io_num, int tx_buffer_size, int rx_buffer_size, int queue_size, int intr_alloc_flags, int baud_rate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits, uart_hw_flowcontrol_t flow_ctrl, uint8_t rx_flow_ctrl_thresh, uart_sclk_t source_clk, uint16_t sen_id);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t pms1003_free_desc(pms1003_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t pms1003_init(pms1003_t *dev);

/**
 * @brief Soft-reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t pms1003_reset(pms1003_t *dev);

esp_err_t pms1003_measure(pms1003_t *dev, pms1003_raw_data_t *raw);

/**
 * @brief High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement
 * 2. Waits until measurement results are available
 * 3. Returns the results
 *
 * @note The function delays the calling task up to 1.1 s to wait for
 *       the measurement results. This might lead to problems when the function
 *       is called from a software timer callback function.
 *
 * @param dev         Device descriptor
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t pms1003_get_raw_data(pms1003_t *dev, pms1003_raw_data_t *raw);

/**
 * @brief Start the measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t pms1003_start_measurement(pms1003_t *dev);

esp_err_t pms1003_set_data_mode(pms1003_t *dev, pms1003_mode_type_t data_mode);
esp_err_t pms1003_set_sleep_mode(pms1003_t *dev, pms1003_sleep_type_t sleep_mode);
esp_err_t pms1003_toggle_sleep_mode(pms1003_t *dev);

/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability and heater setting.
 * Once a measurement is started with function ::pms1003_start_measurement()
 * the user task can use this duration in RTOS ticks directly to wait
 * with function `vTaskDelay()` until the measurement results can be fetched.
 *
 * @param dev       Device descriptor
 * @return          Measurement duration given in RTOS ticks
 */
size_t pms1003_get_measurement_duration(pms1003_t *dev);

/**
 * @brief Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following.
 *
 *      data[0] = Temperature MSB
 *      data[1] = Temperature LSB
 *      data[2] = Temperature CRC
 *      data[3] = Humidity MSB
 *      data[4] = Humidity LSB
 *      data[5] = Humidity CRC
 *
 * In case that there are no new data that can be read, the function fails.
 *
 * @param dev      Device descriptor
 * @param[out] raw Byte array in which raw data are stored
 * @return         `ESP_OK` on success
 */
esp_err_t pms1003_get_raw_data(pms1003_t *dev, pms1003_raw_data_t *raw);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw              Byte array that contains raw data
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_compute_values(pms1003_t *dev,pms1003_raw_data_t *raw);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::pms1003_get_raw_data() and function
 * ::pms1003_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_get_results(pms1003_t *dev);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_toggle_sleep_mode(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t pms1003_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __PMS1003_H__ */
