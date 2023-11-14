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


#ifndef __VEDIRECT_H__
#define __VEDIRECT_H__

#include <stdbool.h>
#include <esp_err.h>
#include <sensor_handler.h>
#include <uartdev.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VED_LIB_VERSION_MAJOR 1
#define VED_LIB_VERSION_MINOR 0
#define VED_LIB_VERSION_PATCH 0
#define VED_LIB_VERSION  (VED_LIB_VERSION_MAJOR << 16)|(VED_LIB_VERSION_MINOR << 8)|VED_LIB_VERSION_PATCH

#define LIB_VERSION_VED "V1.0"

#define VED_LABELS_NR          28
#define VED_DATA_LABEL_LENGTH  9
#define VED_DATA_VAL_LENGTH    24


typedef enum{
  VED_DECODE_READ_STATE_WAIT = 0,
  VED_DECODE_READ_STATE_START1,
  VED_DECODE_READ_STATE_START2,
  VED_DECODE_READ_STATE_LABEL,
  VED_DECODE_READ_STATE_DATA
}ved_decode_state_t;

typedef enum{
  VED_LABEL_TYPE_OUT=0,
  VED_LABEL_TYPE_INFO,
  VED_LABEL_TYPE_UNDEF
}ved_label_type_t;

typedef enum {
  VED_DATA_MODE_PASSIVE = 0,    /**Passive mode */
  VED_DATA_MODE_ACTIVE,         /**Active mode */
}ved_mode_type_t;

typedef enum {  // Only works on active mode
  VED_SLEEP_MODE_SLEEP = 0,     /**acquiring data */
  VED_SLEEP_MODE_AWAKE,         /**data acquisition paused */
  VED_SLEEP_MODE_MAX            /**Invalid mode for initialization */
}ved_sleep_type_t;

typedef enum {
  VED_DEV_BMV60X = 0,
  VED_DEV_BMV70X,
  VED_DEV_BMV71X,
  VED_DEV_MPPT,
  VED_DEV_PHOENIX_INV,
  VED_DEV_PHOENIX_CHG,
  VED_DEV_SMART_BUCKBOSST
}ved_dev_model_t;

/**
 * Device info.
 */
typedef struct {
  uint16_t fw;    // Firmware version
  char bmv[CONFIG_VED_MAX_MODEL_DESC];    // Model description (deprecated)
} ved_dev_inf_t;

/**
 * Device config.
 */
typedef struct {
  ved_dev_mode_type_t data_mode;
} ved_dev_conf_t;

/**
 * Device status.
 */
typedef struct {
  ved_dev_sleep_type_t sleep_mode;
} ved_dev_status_t;

/**
 * Device descriptor
 */
typedef struct {
  uint64_t meas_start_time;
  uart_dev_t uart_dev;
  ved_dev_decode_state_t dstate;
  TaskHandle_t event_task;
  ved_dev_inf_t info;
  ved_dev_conf_t config;
  ved_dev_status_t status;
  ved_dev_model_t dev_model;
  sensor_t sen;
} ved_dev_t;

static const char ved_dev_out_labels[VED_OUT_MAX][VED_DATA_LABEL_LENGTH] = {
	"V",		    // Main or channel 1 (battery) voltage (mv)
	"VS",		    // Auxiliary (starter) voltage (mv)
	"VM",		    // Mid-point voltage of the battery bank (mv)
	"DM",		    // Auxiliary (starter) voltage (mv)
  "VPV",		  // Panel voltage (mv)
  "PPV",		  // Panel power (W)
	"I",		    // Main or channel 1 battery current (mA)
  "I2",		    // Channel 2 battery current (mA)
  "I3",		    // Channel 3 battery current (mA)
  "IL",		    // Load current (mA)
  "LOAD",     // Load output state (bool)
  "T",		    // Battery temperature (°C)
  "P",		    // Instantaneous power (Watt)
	"CE",		    // Consumed Amp Hours (mAh)
	"SOC",		  // State-of-charge (%)
	"TTG",		  // Time-to-go (min)
	"Alarm",	  // Alarm condition active
	"Relay",	  // Relay state
	"H1",		    // Depth of the deepest discharge (mAh)
	"H2",		    // Depth of the last discharge (mAh)
	"H3",		    // Depth of the average discharge (mAh)
	"H4",		    // Number of charge cycles
	"H5",		    // Number of full discharges
	"H6",		    // Cumulative Amp Hours drawn (mAh)
	"H7",		    // Minimum main (battery) voltage (mV)
	"H8",		    // Maximum main (battery) voltage (mV)
	"H9",		    // Number of seconds since last full charge (sec)
	"H10",		  // Number of automatic synchronizations
	"H11",		  // Number of low main voltage alarms
	"H12",		  // Number of high main voltage alarms
	"H13",		  // Number of low auxiliary voltage alarms
	"H14",		  // Number of high auxiliary voltage alarms
	"H15",		  // Minimum auxiliary (battery) voltage (mV)
	"H16",		  // Maximum auxiliary (battery) voltage (mV)
  "H17",		  // Amount of discharged energy (BMV) / Amount of produced energy (DC monitor) (0.01kWh)
  "H18",		  // Amount of charged energy (BMV) / Amount of consumed energy (DC monitor) (0.01kWh)
  "H19",		  // Yield total (user resettable counter) (0.01 kWh)
  "H20",		  // Yield today (0.01 kWh)
  "H21",		  // Maximum power today (W)
  "H22",		  // Yield yesterday (0.01 kWh)
  "H23",		  // Maximum power yesterday (W)
	"AC_OUT_V", // AC output voltage (0.01 V)
	"AC_OUT_I", // AC output current (0.1 A)
	"AC_OUT_S", // AC output apparent power (VA)
	"DC_IN_V",  // DC input voltage (0.01 V)
	"DC_IN_I",  // DC input current (0.1 A)
	"DC_IN_P",  // DC input power (W)
};
static const char ved_dev_info_ved_labels[VED_INF_MAX][VED_DATA_LABEL_LENGTH] = {
	"AR",		    // Alarm reason
  "OR",       // Off reason
  "ERR",	    // Error code
  "CS",       // State of operation
  "BMV",	    // Model description (deprecated)
	"FW",		    // Firmware version (16 bit)
	"FWE",	    // Firmware version (24 bit)
	"PID",	    // Product ID
	"SER#",	    // Serial number
	"HSDS",	    // Day sequence number (0...364)
	"MODE",	    // Device mode
	"WARN",     // Warning reason
	"MPPT",     // Tracker operation mode
	"MON",      // DC monitor mode
  "Checksum"  // Data integrety check
};
/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param addr      VED address
 * @param port      UART port
 * @param tx_gpio   TX GPIO
 * @param rx_gpio   RX GPIO
 * @return          ESP_OK` on success
 */
esp_err_t ved_dev_init_desc(ved_dev_t *dev, uart_port_t port, gpio_num_t tx_gpio, \
  gpio_num_t rx_gpio, int tx_buffer_size, \
  int rx_buffer_size, int queue_size, \
  uart_sclk_t source_clk, uint16_t sen_id);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t ved_dev_free_desc(ved_dev_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t ved_dev_init(ved_dev_t *dev);

/**
 * @brief Soft-reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t ved_dev_reset(ved_dev_t *dev);

esp_err_t ved_dev_measure(ved_dev_t *dev, ved_dev_raw_data_t *raw);

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
esp_err_t ved_dev_get_raw_data(ved_dev_t *dev, ved_dev_raw_data_t *raw);

/**
 * @brief Start the measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t ved_dev_start_measurement(ved_dev_t *dev);


/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability and heater setting.
 * Once a measurement is started with function ::ved_dev_start_measurement()
 * the user task can use this duration in RTOS ticks directly to wait
 * with function `vTaskDelay()` until the measurement results can be fetched.
 *
 * @param dev       Device descriptor
 * @return          Measurement duration given in RTOS ticks
 */
size_t ved_dev_get_measurement_duration(ved_dev_t *dev);

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
esp_err_t ved_dev_get_raw_data(ved_dev_t *dev, ved_dev_raw_data_t *raw);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw              Byte array that contains raw data
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_compute_values(ved_dev_t *dev,ved_dev_raw_data_t *raw);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::ved_dev_get_raw_data() and function
 * ::ved_dev_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_get_results(ved_dev_t *dev);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_toggle_sleep_mode(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t ved_dev_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __VED_H__ */
