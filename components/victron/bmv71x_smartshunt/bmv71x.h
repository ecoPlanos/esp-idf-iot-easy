/*
 * Copyright (c) 2023 ecoPlanos <geral@ecoplanos.pt>
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


#ifndef __BMV71X_H__
#define __BMV71X_H__

#include <stdbool.h>
#include <esp_err.h>
#include <sensor_handler.h>
#include <uartdev.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMV71X_LIB_VERSION_MAJOR 1
#define BMV71X_LIB_VERSION_MINOR 0
#define BMV71X_LIB_VERSION_PATCH 0
#define BMV71X_LIB_VERSION  (BMV71X_LIB_VERSION_MAJOR << 16)|(BMV71X_LIB_VERSION_MINOR << 8)|BMV71X_LIB_VERSION_PATCH

#define LIB_VERSION_BMV71X "V1.0"

#define BMV71X_VED_LABELS_NR      28
#define BMV71X_DATA_LABEL_LENGTH  9
#define BMV71X_DATA_VAL_LENGTH    24

typedef enum {
  BMV71X_OUT_V=0,
  BMV71X_OUT_VS,
  BMV71X_OUT_I,
  BMV71X_OUT_CE,
  BMV71X_OUT_SOC,
  BMV71X_OUT_TIG,
  BMV71X_OUT_Alarm,
  BMV71X_OUT_Relay,
  BMV71X_OUT_AR,
  BMV71X_OUT_H1,
  BMV71X_OUT_H2,
  BMV71X_OUT_H3,
  BMV71X_OUT_H4,
  BMV71X_OUT_H5,
  BMV71X_OUT_H6,
  BMV71X_OUT_H7,
  BMV71X_OUT_H8,
  BMV71X_OUT_H9,
  BMV71X_OUT_H10,
  BMV71X_OUT_H11,
  BMV71X_OUT_H12,
  BMV71X_OUT_H13,
  BMV71X_OUT_H14,
  BMV71X_OUT_H15,
  BMV71X_OUT_H16,
  BMV71X_OUT_MAX,
}bmv71x_out_ids_t;

typedef enum {
  BMV71X_INF_BMV=0,
  BMV71X_INF_FW,
  BMV71X_INF_CHECKSUM,
  BMV71X_INF_MAX
}bmv71x_info_ids_t;

typedef enum {
  ALARM_LOW_VOLTAGE=0x01,
  ALARM_HIGH_VOLTAGE=0x02,
  ALARM_LOW_SOC= 0x04,
  ALARM_LOW_STARTER_VOLTAGE=0x08,
  ALARM_HIGH_STARTER_VOLTAGE=0x10
}bmv71x_alarm_codes_t;

typedef enum{
  BMV71X_DECODE_READ_STATE_WAIT = 0,
  BMV71X_DECODE_READ_STATE_START1,
  BMV71X_DECODE_READ_STATE_START2,
  BMV71X_DECODE_READ_STATE_LABEL,
  BMV71X_DECODE_READ_STATE_DATA
}bmv71x_decode_state_t;

typedef enum{
  BMV71X_LABEL_TYPE_OUT=0,
  BMV71X_LABEL_TYPE_INFO,
  BMV71X_LABEL_TYPE_UNDEF
}bmv71x_label_type_t;

typedef enum {
  BMV71X_DATA_MODE_PASSIVE = 0,    /**Passive mode */
  BMV71X_DATA_MODE_ACTIVE,         /**Active mode */
}bmv71x_mode_type_t;

typedef enum {  // Only works on active mode
  BMV71X_SLEEP_MODE_SLEEP = 0,     /**acquiring data */
  BMV71X_SLEEP_MODE_AWAKE,         /**data acquisition paused */
  BMV71X_SLEEP_MODE_MAX            /**Invalid mode for initialization */
}bmv71x_sleep_type_t;

typedef struct {
  int32_t v;
  int32_t vs;
  int32_t i;
  int32_t ce;
  int32_t soc;
  int32_t tig;
  int32_t alarm;
  int32_t relay;
  int32_t ar;
  int32_t h1;
  int32_t h2;
  int32_t h3;
  int32_t h4;
  int32_t h5;
  int32_t h6;
  int32_t h7;
  int32_t h8;
  int32_t h9;
  int32_t h10;
  int32_t h11;
  int32_t h12;
  int32_t h13;
  int32_t h14;
  int32_t h15;
  int32_t h16;
} bmv71x_raw_data_t;

/**
 * Device info.
 */
typedef struct {
  uint16_t fw;    // Firmware version
  char bmv[CONFIG_BMV71X_MAX_MODEL_DESC];    // Model description (deprecated)
} bmv71x_inf_t;

/**
 * Device config.
 */
typedef struct {
  bmv71x_mode_type_t data_mode;
} bmv71x_conf_t;

/**
 * Device status.
 */
typedef struct {
  bmv71x_sleep_type_t sleep_mode;
} bmv71x_status_t;

/**
 * Device descriptor
 */
typedef struct {
  uart_dev_t uart_dev;
  bmv71x_decode_state_t dstate;
  TaskHandle_t event_task;
  uint64_t meas_start_time;
  bmv71x_inf_t info;
  bmv71x_conf_t config;
  bmv71x_status_t status;
  sensor_t sen;
} bmv71x_t;

static const char bmv71x_out_ved_labels[BMV71X_OUT_MAX][BMV71X_DATA_LABEL_LENGTH] = {
	"V",		  // Main or channel 1 (battery) voltage (mv)
	"VS",		  // Auxiliary (starter) voltage (mv)
	"VM",		  // Mid-point voltage of the battery bank (mv)
	"DM",		  // Auxiliary (starter) voltage (mv)
	"I",		  // Main or channel 1 battery current (mA)
  "T",		  // Battery temperature (°C)
  "P",		  // Instantaneous power (Watt)
	"CE",		  // Consumed Amp Hours (mAh)
	"SOC",		// State-of-charge (%)
	"TIG",		// Time-to-go (min)
	"Alarm",	// Alarm condition active
	"Relay",	// Relay state
	"AR",		  // Alarm reason
	"H1",		  // Depth of the deepest discharge (mAh)
	"H2",		  // Depth of the last discharge (mAh)
	"H3",		  // Depth of the average discharge (mAh)
	"H4",		  // Number of charge cycles
	"H5",		  // Number of full discharges
	"H6",		  // Cumulative Amp Hours drawn (mAh)
	"H7",		  // Minimum main (battery) voltage (mV)
	"H8",		  // Maximum main (battery) voltage (mV)
	"H9",		  // Number of seconds since last full charge (sec)
	"H10",		// Number of automatic synchronizations
	"H11",		// Number of low main voltage alarms
	"H12",		// Number of high main voltage alarms
	"H13",		// Number of low auxiliary voltage alarms
	"H14",		// Number of high auxiliary voltage alarms
	"H15",		// Minimum auxiliary (battery) voltage (mV)
	"H16",		// Maximum auxiliary (battery) voltage (mV)
};
static const char bmv71x_info_ved_labels[BMV71X_INF_MAX][BMV71X_DATA_LABEL_LENGTH] = {
  "BMV",		  // Model description (deprecated)
	"FW",		    // Firmware version (16 bit)
  "Checksum"  // Data integrety check
};
/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param addr      BMV71X address
 * @param port      UART port
 * @param tx_gpio   TX GPIO
 * @param rx_gpio   RX GPIO
 * @return          ESP_OK` on success
 */
esp_err_t bmv71x_init_desc(bmv71x_t *dev, uart_port_t port, gpio_num_t tx_gpio, \
  gpio_num_t rx_gpio, int tx_buffer_size, \
  int rx_buffer_size, int queue_size, \
  uart_sclk_t source_clk, uint16_t sen_id);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t bmv71x_free_desc(bmv71x_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t bmv71x_init(bmv71x_t *dev);

/**
 * @brief Soft-reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t bmv71x_reset(bmv71x_t *dev);

esp_err_t bmv71x_measure(bmv71x_t *dev, bmv71x_raw_data_t *raw);

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
esp_err_t bmv71x_get_raw_data(bmv71x_t *dev, bmv71x_raw_data_t *raw);

/**
 * @brief Start the measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t bmv71x_start_measurement(bmv71x_t *dev);


/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability and heater setting.
 * Once a measurement is started with function ::bmv71x_start_measurement()
 * the user task can use this duration in RTOS ticks directly to wait
 * with function `vTaskDelay()` until the measurement results can be fetched.
 *
 * @param dev       Device descriptor
 * @return          Measurement duration given in RTOS ticks
 */
size_t bmv71x_get_measurement_duration(bmv71x_t *dev);

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
esp_err_t bmv71x_get_raw_data(bmv71x_t *dev, bmv71x_raw_data_t *raw);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw              Byte array that contains raw data
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_compute_values(bmv71x_t *dev,bmv71x_raw_data_t *raw);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::bmv71x_get_raw_data() and function
 * ::bmv71x_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_get_results(bmv71x_t *dev);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_toggle_sleep_mode(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t bmv71x_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __BMV71X_H__ */
