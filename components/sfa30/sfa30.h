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
 * @file sfa30.h
 * @defgroup sfa30 sfa30
 * @{
 *
 * ESP-IDF driver for Sensirion SFA30 digital temperature and humidity sensor
 *
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SFA30_H__
#define __SFA30_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>
#include <sensor_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SFA30_LIB_VERSION_MAJOR 1
#define SFA30_LIB_VERSION_MINOR 0
#define SFA30_LIB_VERSION_PATCH 0
#define SFA30_LIB_VERSION  (SFA30_LIB_VERSION_MAJOR << 16)|(SFA30_LIB_VERSION_MINOR << 8)|SFA30_LIB_VERSION_PATCH

#define SFA30_I2C_ADDRESS 0x5D

#define SFA30_RAW_DATA_SIZE 9
#define SFA30_MARKING_DATA_SIZE 48

enum {
  SFA30_OUT_HCHO_ID = 0,
  SFA30_OUT_RH_ID,
  SFA30_OUT_TEMP_ID
};

typedef uint8_t sfa30_raw_data_t[SFA30_RAW_DATA_SIZE];


/**
 * Possible heater modes
 */
typedef enum
{
  SFA30_STATUS_COMMAND              =  0b0100000000000000,      /**< Command status '0': last command executed successfully '1': last command not processed. It was either invalid, failed the integrated command checksum */
  SFA30_STATUS_WRITE_DATA_CHECKSUM  =  0b1000000000000000,      /**< Write data checksum status '0': checksum of last write transfer was correct '1': checksum of last write transfer failed */
} sfa30_status_t;

/**
 * Device info.
 */
typedef struct
{
    uint8_t pack_id;    // Package Identification
    uint8_t dev_id;     // Device Identification
    sfa30_status_t status;
} sfa30_inf_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;            //!< I2C device descriptor
    uint32_t serial;              //!< device serial number
    bool meas_started;            //!< indicates whether measurement started
    uint64_t meas_start_time;     //!< measurement start time in us
    sfa30_inf_t info;
    bool measurement_running;
    sensor_t sen;
} sfa30_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param addr      SFA30 address
 * @param port      I2C port
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return          ESP_OK` on success
 */
esp_err_t sfa30_init_desc(sfa30_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sfa30_free_desc(sfa30_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sfa30_init(sfa30_t *dev);

/**
 * @brief Soft-reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sfa30_reset(sfa30_t *dev);

/**
 * @brief Read the device marking string from the device.
 *
 * @param device_marking ASCII string containing the serial number. The
 * string has the null-termination character included.
 *
 * @return 0 on success, an error code otherwise
 */
esp_err_t sfa30_get_device_marking(sfa30_t *dev, uint8_t device_marking[]);

/**
 * @brief High level measurement function
 *
 * For convenience this function comprises all three steps to perform
 * one measurement in only one function:
 *
 * 1. Starts a measurement
 * 2. Waits using `vTaskDelay()` until measurement results are available
 * 3. Returns the results in kind of floating point sensor values
 *
 * This function is the easiest way to use the sensor. It is most suitable
 * for users that don't want to have the control on sensor details.
 *
 * @note The function delays the calling task up to 1.1 s to wait for
 *       the measurement results. This might lead to problems when the function
 *       is called from a software timer callback function.
 *
 * @param dev         Device descriptor
 * @param hcho        HCHO consentration
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sfa30_measure(sfa30_t *dev, float *hcho, float *humidity, float *temperature);

/**
 * @brief Start the measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sfa30_start_measurement(sfa30_t *dev);

/**
 * @brief Issue a command to retrieve data from sensor.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sfa30_get_data(sfa30_t *dev);

/**
 * @brief Stop the periodic measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sfa30_stop_measurement(sfa30_t *dev);

/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability and heater setting.
 * Once a measurement is started with function ::sfa30_start_measurement()
 * the user task can use this duration in RTOS ticks directly to wait
 * with function `vTaskDelay()` until the measurement results can be fetched.
 *
 * @param dev       Device descriptor
 * @return          Measurement duration given in RTOS ticks
 */
size_t sfa30_get_measurement_duration(sfa30_t *dev);

/**
 * @brief Read measurement results from sensor as raw data
 *
 * The function read measurement results from the sensor, checks the CRC
 * checksum and stores them in the byte array as following.
 *
 *      data[0] = HCHO MSB
 *      data[1] = HCHO LSB
 *      data[2] = HCHO CRC
 *      data[3] = Humidity MSB
 *      data[4] = Humidity LSB
 *      data[5] = Humidity CRC
 *      data[6] = Temperature MSB
 *      data[7] = Temperature LSB
 *      data[8] = Temperature CRC
 *
 * In case that there are no new data that can be read, the function fails.
 *
 * @param dev      Device descriptor
 * @param[out] raw Byte array in which raw data are stored
 * @return         `ESP_OK` on success
 */
esp_err_t sfa30_get_raw_data(sfa30_t *dev, sfa30_raw_data_t raw);

/**
 * @brief Computes sensor values from raw data
 *
 * @param dev              SFA30 device descriptor
 * @param raw_data         Byte array that contains raw data
 * @param[out] hcho        HCHO consentration
 * @param[out] humidity    Humidity in percent
 * @param[out] temperature Temperature in degree Celsius
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_compute_values(sfa30_t *dev, sfa30_raw_data_t raw_data, float *hcho, float *humidity, float *temperature);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::sfa30_get_raw_data() and function
 * ::sfa30_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev              Device descriptor
 * @param[out] hcho        HCHO concentration
 * @param[out] humidity    Humidity in percent
 * @param[out] temperature Temperature in degree Celsius
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_get_results(sfa30_t *dev, float *hcho, float *humidity, float *temperature);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t sfa30_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SFA30_H__ */
