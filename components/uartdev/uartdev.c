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

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <string.h>
#include <driver/uart.h>
#include "uartdev.h"

static const char *TAG = "uartdev";

typedef struct {
    SemaphoreHandle_t lock;
    uart_config_t config;
    bool installed;
} uart_port_state_t;

static uart_port_state_t states[UART_NUM_MAX];

#if CONFIG_UARTDEV_NOLOCK
#define SEMAPHORE_TAKE(port)
#else
#define SEMAPHORE_TAKE(port) do { \
        if (!xSemaphoreTake(states[port].lock, pdMS_TO_TICKS(CONFIG_UARTDEV_TIMEOUT))) \
        { \
            ESP_LOGE(TAG, "Could not take port mutex %d", port); \
            return ESP_ERR_TIMEOUT; \
        } \
        } while (0)
#endif

#if CONFIG_UARTDEV_NOLOCK
#define SEMAPHORE_GIVE(port)
#else
#define SEMAPHORE_GIVE(port) do { \
        if (!xSemaphoreGive(states[port].lock)) \
        { \
            ESP_LOGE(TAG, "Could not give port mutex %d", port); \
            return ESP_FAIL; \
        } \
        } while (0)
#endif

esp_err_t uartdev_init()
{
    memset(states, 0, sizeof(states));

#if !CONFIG_UARTDEV_NOLOCK
    for (int i = 0; i < UART_NUM_MAX; i++)
    {
        states[i].lock = xSemaphoreCreateMutex();
        if (!states[i].lock)
        {
            ESP_LOGE(TAG, "Could not create port mutex %d", i);
            return ESP_FAIL;
        }
    }
#endif

    return ESP_OK;
}

esp_err_t uartdev_done()
{
    for (int i = 0; i < UART_NUM_MAX; i++)
    {
        if (!states[i].lock) continue;

        if (states[i].installed)
        {
            SEMAPHORE_TAKE(i);
            uart_driver_delete(i);
            states[i].installed = false;
            SEMAPHORE_GIVE(i);
        }
#if !CONFIG_UARTDEV_NOLOCK
        vSemaphoreDelete(states[i].lock);
#endif
        states[i].lock = NULL;
    }
    return ESP_OK;
}

esp_err_t uart_dev_create_mutex(uart_dev_t *dev)
{
#if !CONFIG_UARTDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[UART port %d] creating mutex", dev->port);

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex)
    {
        ESP_LOGE(TAG, "[UART port %d] Could not create device mutex", dev->port);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

esp_err_t uart_dev_delete_mutex(uart_dev_t *dev)
{
#if !CONFIG_UARTDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[UART port %d] deleting mutex", dev->port);

    vSemaphoreDelete(dev->mutex);
#endif
    return ESP_OK;
}

esp_err_t uart_dev_take_mutex(uart_dev_t *dev)
{
#if !CONFIG_UARTDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[UART port %d] taking mutex", dev->port);

    if (!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(CONFIG_UARTDEV_TIMEOUT)))
    {
        ESP_LOGE(TAG, "[UART port %d] Could not take device mutex", dev->port);
        return ESP_ERR_TIMEOUT;
    }
#endif
    return ESP_OK;
}

esp_err_t uart_dev_give_mutex(uart_dev_t *dev)
{
#if !CONFIG_UARTDEV_NOLOCK
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGV(TAG, "[UART port %d] giving mutex", dev->port);

    if (!xSemaphoreGive(dev->mutex))
    {
        ESP_LOGE(TAG, "[UART port %d] Could not give device mutex", dev->port);
        return ESP_FAIL;
    }
#endif
    return ESP_OK;
}

inline static bool cfg_equal(const uart_config_t *a, const uart_config_t *b)
{
    return a->baud_rate == b->baud_rate
        && a->data_bits == b->data_bits
        && a->parity == b->parity
        && a->stop_bits == b->stop_bits
        && a->flow_ctrl == b->flow_ctrl
        && a->source_clk == b->source_clk;
}

static esp_err_t uart_setup_port(const uart_dev_t *dev)
{
    if (dev->port >= UART_NUM_MAX) return ESP_ERR_INVALID_ARG;

    esp_err_t res;
    if (!cfg_equal(&dev->cfg, &states[dev->port].config))
    {
        ESP_LOGD(TAG, "Reconfiguring UART driver on port %d", dev->port);
        uart_config_t temp;
        memcpy(&temp, &dev->cfg, sizeof(uart_config_t));

        // Driver reinstallation
        if (states[dev->port].installed)
            uart_driver_delete(dev->port);

        if ((res = uart_driver_install(dev->port, dev->rx_buffer_size, dev->tx_buffer_size, dev->queue_size, dev->queue, dev->intr_alloc_flags)) != ESP_OK)
          return res;
        if ((res = uart_param_config(dev->port, &temp)) != ESP_OK)
          return res;
        if ((res = uart_set_pin(dev->port, dev->tx_io_num, dev->rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK)
          return res;

        states[dev->port].installed = true;

        memcpy(&states[dev->port].config, &temp, sizeof(uart_config_t));
        ESP_LOGD(TAG, "UART driver successfully reconfigured on port %d", dev->port);
    }
    return ESP_OK;
}

// esp_err_t uart_dev_read(const uart_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
// {
//     if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;
//
//     SEMAPHORE_TAKE(dev->port);
//     // if(dev->port)
//     esp_err_t res = uart_setup_port(dev);
//     if (res == ESP_OK)
//     {
//         uart_cmd_handle_t cmd = uart_cmd_link_create();
//         if (out_data && out_size)
//         {
//             uart_master_start(cmd);
//             uart_master_write_byte(cmd << 1, true);
//             uart_master_write(cmd, (void *)out_data, out_size, true);
//         }
//         uart_master_start(cmd);
//         uart_master_write_byte(cmd, (dev->addr << 1) | 1, true);
//         uart_master_read(cmd, in_data, in_size, UART_MASTER_LAST_NACK);
//         uart_master_stop(cmd);
//
//         res = uart_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(CONFIG_UARTDEV_TIMEOUT));
//         if (res != ESP_OK)
//             ESP_LOGE(TAG, "Could not read from device [UART port %d]: %d", dev->port, res);
//
//         uart_cmd_link_delete(cmd);
//     }
//
//     SEMAPHORE_GIVE(dev->port);
//     return res;
// }

int uart_dev_write(const uart_dev_t *dev, const uint8_t* data)
{
  if (!dev) return ESP_ERR_INVALID_ARG;
  const int len = strlen((char*)data);
  SEMAPHORE_TAKE(dev->port);
  const int txBytes = uart_write_bytes(dev->port, data, len);
  SEMAPHORE_GIVE(dev->port);
  ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
  return txBytes;
}

void rx_task(void *arg)
{
  uart_dev_t *dev = (uart_dev_t *)arg;
    // static const char *TAG = "RX_TASK";
    // esp_log_level_set(TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(dev->rx_buffer_size+1);
    while (1) {
        const int rxBytes = uart_read_bytes(dev->port, data, dev->rx_buffer_size, dev->delay_ms / portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}
