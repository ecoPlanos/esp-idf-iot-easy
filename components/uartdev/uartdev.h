/*
 * Copyright (c) 2021 Ecoplanos: Automação e Sistemas Integrados, Lda.
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

#ifndef __UARTDEV_H__
#define __UARTDEV_H__

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART_DEV_RX_BUF_SIZE CONFIG_UART_DEV_RX_BUF_SIZE

/**
 * UART device descriptor
 */
typedef struct {
  int rx_buffer_size;
  int tx_buffer_size;
  int queue_size;
  QueueHandle_t queue;
  int intr_alloc_flags;

  int tx_io_num;
  int rx_io_num;
  int rts_io_num;
  int cts_io_num;

  uint32_t delay_ms;        // Delay to check serial port multiple times

  uart_port_t port;         //!< UART port number
  uart_config_t cfg;        //!< UART driver configuration
  SemaphoreHandle_t mutex; //!< Device mutex
} uart_dev_t;

/**
 * @brief Init library
 *
 * The function must be called before any other
 * functions of this library.
 *
 * @param rx_buffer_size Buffer allocation size
 * @return ESP_OK on success
 */
esp_err_t uartdev_init();

/**
 * @brief Finish work with library
 *
 * Uninstall uart drivers.
 *
 * @return ESP_OK on success
 */
esp_err_t uartdev_done();

/**
 * @brief Create mutex for device descriptor
 *
 * This function does nothing if option CONFIG_UARTDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t uart_dev_create_mutex(uart_dev_t *dev);

/**
 * @brief Delete mutex for device descriptor
 *
 * This function does nothing if option CONFIG_UARTDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t uart_dev_delete_mutex(uart_dev_t *dev);

/**
 * @brief Take device mutex
 *
 * This function does nothing if option CONFIG_UARTDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t uart_dev_take_mutex(uart_dev_t *dev);

/**
 * @brief Give device mutex
 *
 * This function does nothing if option CONFIG_UARTDEV_NOLOCK is enabled.
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t uart_dev_give_mutex(uart_dev_t *dev);

/**
 * @brief Read from slave device
 *
 * Issue a send operation of \p out_data register address, followed by reading \p in_size bytes
 * from slave into \p in_data .
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_data Pointer to data to send if non-null
 * @param out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t uart_dev_read(const uart_dev_t *dev, const void *out_data,
        size_t out_size, uint32_t timeout);

/**
 * @brief Write to slave device
 *
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param data Data string to send
 * @return bytes written
 */
esp_err_t uart_dev_write(const uart_dev_t *dev, const uint8_t* data, size_t data_size);

/**
 * @brief Receive from slave device
 *
 * Task to receive newest data from serial port.
 * Function is thread-safe.
 *
 * @param arg uart_dev_t *device
 */
void rx_task(void *arg);

#define UART_DEV_TAKE_MUTEX(dev) do { \
        esp_err_t __ = uart_dev_take_mutex(dev); \
        if (__ != ESP_OK) return __;\
    } while (0)

#define UART_DEV_GIVE_MUTEX(dev) do { \
        esp_err_t __ = uart_dev_give_mutex(dev); \
        if (__ != ESP_OK) return __;\
    } while (0)

#define UART_DEV_CHECK(dev, X) do { \
        esp_err_t ___ = X; \
        if (___ != ESP_OK) { \
            UART_DEV_GIVE_MUTEX(dev); \
            return ___; \
        } \
    } while (0)

#define UART_DEV_CHECK_LOGE(dev, X, msg, ...) do { \
        esp_err_t ___ = X; \
        if (___ != ESP_OK) { \
            UART_DEV_GIVE_MUTEX(dev); \
            ESP_LOGE(TAG, msg, ## __VA_ARGS__); \
            return ___; \
        } \
    } while (0)

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __UARTDEV_H__ */
