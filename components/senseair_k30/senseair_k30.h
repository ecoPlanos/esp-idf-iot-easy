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

#ifndef __SENSEAIR_K30_H__
#define __SENSEAIR_K30_H__

#include <driver/gpio.h>
#include <i2cdev.h>
#include <esp_err.h>
#include <sensor_handler.h>

#ifdef __cplusplus
extern "C" {
#endif

#define K30_LIB_VERSION_MAJOR 1
#define K30_LIB_VERSION_MINOR 0
#define K30_LIB_VERSION_PATCH 2
#define K30_LIB_VERSION  (K30_LIB_VERSION_MAJOR << 16)|(K30_LIB_VERSION_MINOR << 8)|K30_LIB_VERSION_PATCH

#define K30_CONF_FILE_NAME "conf.cfg"
#define K30_DATA_FILE_NAME "dat.txt"
#define K30_FILE_DIR "K30"
#define K30_CONF_FILE_PATH K30_FILE_DIR "/" K30_CONF_FILE_NAME
#define K30_DATA_FILE_PATH K30_FILE_DIR "/" K30_DATA_FILE_NAME

#define K30_I2C_ADDR   0x68 // K30 has only one i2c address but can be changed writing to eeprom.

enum {
  K30_OUT_CO2_ID = 0,
  K30_OUT_TEMP_ID,
  K30_OUT_RH_ID
};

/**
 * Power status. The sensor measures only if ALS an Power is on.
 */
typedef enum {
  K30_POWER_OFF = 0x00,
  K30_POWER_ON = 0x01 //!< Default
} k30_power_status_t;

/**
 * Device info
 */
typedef enum {
  K30_REGISTER_PACKAGE_PID = 0x11,    // Package Identification
  K30_REGISTER_DEVICE_ID = 0x12,      // Device Identification
  K30_REGISTER_DEVICE_STATUS = 0x13,  // Internal Status
} k30_info_t;

/**
 * Device settings.
 */
typedef struct {
  uint8_t enable_reg;
  uint8_t control_reg;
  uint8_t persistence_reg;
} k30_settings_t;

/**
 * Device info.
 */
typedef struct {
  uint8_t pack_id;    // Package Identification
  uint8_t dev_id;     // Device Identification
  uint8_t status;
} k30_inf_t;

/**
 * Device descriptor.
 */
typedef struct {
  i2c_dev_t i2c_dev;
  k30_settings_t settings;
  k30_inf_t info;
  sensor_t sen;
} k30_t;


/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t k30_init_desc(k30_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint16_t sen_id, char *sen_name);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t k30_free_desc(k30_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t k30_init(k30_t *dev);


/**
 * @brief Get CO2 concentration
 *
 * @param dev Device descriptor
 * @param[out] co2 CO2 concentration
 * @return `ESP_OK`
 */
esp_err_t k30_get_co2(k30_t *dev, float *co2);


/**************************************************************************/
/*!
    @brief  Enables the chip, so it's ready to take readings
*/
/**************************************************************************/
esp_err_t k30_basic_enable(k30_t *dev);

/**************************************************************************/
/*!
    @brief Disables the chip, so it's in power down mode
*/
/**************************************************************************/
esp_err_t k30_basic_disable(k30_t *dev);

/**
 * @brief Start a new measurement
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t k30_iot_sen_start_measurement(void *dev);

/**
 * @brief Get last measurement data
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t k30_iot_sen_get_data(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t k30_iot_sen_sleep_mode_awake(void *dev);

/**
 * @brief Set sensor to sleep mode or awake from sleep
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t k30_iot_sen_sleep_mode_sleep(void *dev);

/**
 * @brief Reset sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t k30_iot_sen_reset(void *dev);

/**
 * @brief Reinitialize sensor
 *
 * @param dev              Device descriptor
 * @return                 `ESP_OK` on success
 */
esp_err_t k30_iot_sen_reinit(void *dev);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __K30_H__
