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
 * @file sht85.h
 * @defgroup sht85 sht85
 * @{
 *
 * ESP-IDF driver for Sensirion SHT85 digital temperature and humidity sensor
 *
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SHT85_H__
#define __SHT85_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>
#include <sensor_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHT85_LIB_VERSION_MAJOR 1
#define SHT85_LIB_VERSION_MINOR 0
#define SHT85_LIB_VERSION_PATCH 0
#define SHT85_LIB_VERSION  (SHT85_LIB_VERSION_MAJOR << 16)|(SHT85_LIB_VERSION_MINOR << 8)|SHT85_LIB_VERSION_PATCH

#define SHT85_I2C_ADDRESS 0x44

#define SHT85_RAW_DATA_SIZE 6*2

typedef enum
{
  SHT85_OUT_TEMP_ID = 0,
  SHT85_OUT_RH_ID
};

typedef uint8_t sht85_raw_data_t[SHT85_RAW_DATA_SIZE];

/**
 * Possible heater modes
 */
typedef enum
{
    SHT85_HEATER_OFF = 0,      /**< Heater is off, default */
    SHT85_HEATER_ON            /**< Heater is off, default */
    // SHT85_HEATER_HIGH_LONG,    /**< High power (~200mW), 1 second pulse */
    // SHT85_HEATER_HIGH_SHORT,   /**< High power (~200mW), 0.1 second pulse */
    // SHT85_HEATER_MEDIUM_LONG,  /**< Medium power (~110mW), 1 second pulse */
    // SHT85_HEATER_MEDIUM_SHORT, /**< Medium power (~110mW), 0.1 second pulse */
    // SHT85_HEATER_LOW_LONG,     /**< Low power (~20mW), 1 second pulse */
    // SHT85_HEATER_LOW_SHORT,    /**< Low power (~20mW), 0.1 second pulse */
} sht85_heater_t;

/**
 * Possible heater modes
 */
typedef enum
{
    SHT85_STATUS_ALERT_PENDING        =  0b0000000000000001,      /**< Alert pending status '0': no pending alerts '1': at least one pending alert */
    SHT85_STATUS_HEATER               =  0b0000000000000100,      /**< Heater status ‘0’ : Heater OFF ‘1’ : Heater ON */
    SHT85_STATUS_RH_TRACKING_ALERT    =  0b0000000000010000,      /**< RH tracking alert ‘0’ : no alert ‘1’ . alert */
    SHT85_STATUS_T_TRACKING_ALERT     =  0b0000000000100000,      /**< T tracking alert ‘0’ : no alert ‘1’ . alert */
    SHT85_STATUS_SYS_RESET_DETECTED   =  0b0000100000000000,      /**< System reset detected '0': no reset detected since last ‘clear status register’ command '1': reset detected (hard reset, soft reset command or supply fail)  */
    SHT85_STATUS_COMMAND              =  0b0100000000000000,      /**< Command status '0': last command executed successfully '1': last command not processed. It was either invalid, failed the integrated command checksum */
    SHT85_STATUS_WRITE_DATA_CHECKSUM  =  0b1000000000000000,      /**< Write data checksum status '0': checksum of last write transfer was correct '1': checksum of last write transfer failed */
} sht85_status_t;

/**
 * Possible repeatability modes
 */
typedef enum
{
    SHT85_HIGH = 0,
    SHT85_MEDIUM,
    SHT85_LOW
} sht85_repeat_t;

/**
 * Device info.
 */
typedef struct
{
    uint8_t pack_id;    // Package Identification
    uint8_t dev_id;     // Device Identification
    sht85_status_t status;
} sht85_inf_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;            //!< I2C device descriptor

    uint32_t serial;              //!< device serial number

    sht85_heater_t heater;        //!< used measurement mode
    sht85_repeat_t repeatability; //!< used repeatability

    bool meas_started;            //!< indicates whether measurement started
    uint64_t meas_start_time;     //!< measurement start time in us
    sht85_inf_t info;
    sensor_t sen;
} sht85_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev       Device descriptor
 * @param addr      SHT85 address
 * @param port      I2C port
 * @param sda_gpio  SDA GPIO
 * @param scl_gpio  SCL GPIO
 * @return          ESP_OK` on success
 */
esp_err_t sht85_init_desc(sht85_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id);

/**
 * @brief Free device descriptor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_free_desc(sht85_t *dev);

/**
 * @brief Initialize sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_init(sht85_t *dev);

/**
 * @brief Soft-reset sensor
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_reset(sht85_t *dev);

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
 * @param temperature Temperature in degree Celsius
 * @param humidity    Humidity in percent
 * @return            `ESP_OK` on success
 */
esp_err_t sht85_measure(sht85_t *dev, float *temperature, float *humidity);

/**
 * @brief Start the measurement.
 *
 * @param dev       Device descriptor
 * @return          `ESP_OK` on success
 */
esp_err_t sht85_start_measurement(sht85_t *dev);

/**
 * @brief Get the duration of a measurement in RTOS ticks.
 *
 * The function returns the duration in RTOS ticks required by the sensor to
 * perform a measurement for the given repeatability and heater setting.
 * Once a measurement is started with function ::sht85_start_measurement()
 * the user task can use this duration in RTOS ticks directly to wait
 * with function `vTaskDelay()` until the measurement results can be fetched.
 *
 * @param dev       Device descriptor
 * @return          Measurement duration given in RTOS ticks
 */
size_t sht85_get_measurement_duration(sht85_t *dev);

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
esp_err_t sht85_get_raw_data(sht85_t *dev, sht85_raw_data_t raw);

/**
 * @brief Computes sensor values from raw data
 *
 * @param raw_data         Byte array that contains raw data
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t sht85_compute_values(sht85_raw_data_t raw_data, float *temperature, float *humidity);

/**
 * @brief Get measurement results in form of sensor values
 *
 * The function combines function ::sht85_get_raw_data() and function
 * ::sht85_compute_values() to get the measurement results.
 *
 * In case that there are no results that can be read, the function fails.
 *
 * @param dev              Device descriptor
 * @param[out] temperature Temperature in degree Celsius
 * @param[out] humidity    Humidity in percent
 * @return                 `ESP_OK` on success
 */
esp_err_t sht85_get_results(sht85_t *dev, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SHT85_H__ */
