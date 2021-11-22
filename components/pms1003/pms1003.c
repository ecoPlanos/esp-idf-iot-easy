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
 * @file pms1003.c
 *
 * ESP-IDF driver for Sensirion PMS1003 digital temperature and humidity sensor
 *
 * Copyright (c) 2021 ecoPlanos <geral@ecoplanos.pt>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_idf_lib_helpers.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <string.h>
#include "pms1003.h"

#define FRAME_LENGTH  28 //(2*13 data bytes + 2 check bytes)
#define ANSWER_LENGTH 32 //Sensor reply with 32 byte length message

static const char *TAG = "pms1003";


typedef enum {
  START_BYTE_1 = 0x42,
  START_BYTE_2 = 0x4d,
  START = 0x424d
} pms1003_start_t;

typedef enum {
  CMD_READ_PASSIVE_MODE = 0xe2,
  CMD_CHANGE_MODE =       0xe1,
  CMD_CHANGE_SLEEP =      0xe4
} pms1003_commands_t;

typedef enum {
  STATE_READ_FRAME_LENGTH =             0x4d,  //Fame length=2x13+2(data+check bytes)
  STATE_READ_PM1_0_CON_UNIT =           0x4d,  //DATA1
  STATE_READ_PM2_5_CON_UNIT =           0x4d,  //DATA2
  STATE_READ_PM10_CON_UNIT =            0x4d,  //DATA3
  STATE_READ_PM1_0_CON_UNIT_ATMOSPHE =  0x4d,  //DATA4
  STATE_READ_PM2_5_CON_UNIT_ATMOSPHE =  0x4d,  //DATA5
  STATE_READ_CON_UNIT_ATMOSPHE =        0x4d,  //DATA6
  STATE_READ_PARTICLE_NR_0_3_UM =       0x4d,  //DATA7
  STATE_READ_PARTICLE_NR_0_5_UM =       0x4d,  //DATA8
  STATE_READ_PARTICLE_NR_1_0_UM =       0x4d,  //DATA9
  STATE_READ_PARTICLE_NR_2_5_UM =       0x4d,  //DATA10
  STATE_READ_PARTICLE_NR_5_0_UM =       0x4d,  //DATA11
  STATE_READ_PARTICLE_NR_10_UM =        0x4d,  //DATA12
  STATE_READ_RESERVED =                 0x4d,  //DATA13
  STATE_READ_CHECK =                    0x4d  //Check code = Start character 1 + Start character 2 + ... + data 13 Low 8 bits

} pms1003_transport_protocol_active_state_t;

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static esp_err_t check_code(uint16_t data[]) {
    uint16_t sum = 0;

    for (size_t i = 0; i < FRAME_LENGTH/2-1; i++) {
      sum += data[i];
    }
    ESP_LOGD(TAG, "CRC sum1: %u",sum);
    sum += START_BYTE_1;
    ESP_LOGD(TAG, "CRC sum2: %u",sum);
    sum += START_BYTE_2;
    ESP_LOGD(TAG, "CRC sum3: %u",sum);
    if (data[FRAME_LENGTH/2] != sum) {
        ESP_LOGE(TAG, "Invalid CRC");
        return ESP_ERR_INVALID_CRC;
    }
  return ESP_OK;
}

static inline esp_err_t get_meas_cmd(uint8_t cmd[]) {
  if(!cmd) return ESP_ERR_INVALID_ARG;
  cmd[0] = START_BYTE_1;
  cmd[1] = START_BYTE_2;
  cmd[2] = CMD_READ_PASSIVE_MODE;
  cmd[3] = 0;
  cmd[4] = 0;
  uint16_t check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
  cmd[5] = check>>8;
  cmd[6] = (check<<8)>>8;
  return ESP_OK;
}

static inline esp_err_t get_change_mode_cmd(uint8_t cmd[], pms1003_mode_type_t mode) {
  if(!cmd) return ESP_ERR_INVALID_ARG;
  cmd[0] = START_BYTE_1;
  cmd[1] = START_BYTE_2;
  cmd[2] = CMD_CHANGE_MODE;
  cmd[3] = 0;
  cmd[4] = mode;
  uint16_t check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
  cmd[5] = check>>8;
  cmd[6] = (check<<8)>>8;
  return ESP_OK;
}
static inline esp_err_t get_change_sleep_cmd(uint8_t cmd[], pms1003_sleep_type_t sleep) {
  if(!cmd) return ESP_ERR_INVALID_ARG;
  cmd[0] = START_BYTE_1;
  cmd[1] = START_BYTE_2;
  cmd[2] = CMD_CHANGE_SLEEP;
  cmd[3] = 0;
  cmd[4] = sleep;
  uint16_t check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
  cmd[5] = check>>8;
  cmd[6] = (check<<8)>>8;
  return ESP_OK;
}

static inline esp_err_t send_cmd_nolock(pms1003_t *dev, uint8_t cmd[])
{
    ESP_LOGD(TAG, "Sending cmd %02x %02x %02x %02x %02x %02x %02x...", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6]);
    return uart_dev_write(&dev->uart_dev, cmd);
}

static inline esp_err_t read_res_nolock(pms1003_t *dev, pms1003_raw_data_t *res)
{
    CHECK(uart_dev_read(&dev->uart_dev, NULL, 0, res, PMS1003_RAW_DATA_SIZE));

    uint8_t *res8 = (uint8_t *)res;
    // ESP_LOGD(TAG, "Got response %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x
    //         %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
    //         res8[0], res8[1], res8[2], res8[3], res8[4], res8[5], res8[6], res8[7], res8[8], res8[9],
    //         res8[10], res8[12], res8[13], res8[14], res8[15], res8[16], res8[17], res8[18], res8[19],
    //         res8[20], res8[21], res8[22], res8[23], res8[24], res8[25], res8[26], res8[27]);
    uint16_t *res16 = (uint16_t *)res;
    // ESP_LOGD(TAG, "Got response %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x",
    //         res16[0], res16[1], res16[2], res16[3], res16[4], res16[5], res16[6], res16[7], res16[8], res16[9], res16[10], res16[12], res16[13], res16[14]);
    return check_code(res16);
}

static esp_err_t read_res(pms1003_t *dev, pms1003_raw_data_t *res)
{
    UART_DEV_TAKE_MUTEX(&dev->uart_dev);
    UART_DEV_CHECK(&dev->uart_dev, read_res_nolock(dev, res));
    UART_DEV_GIVE_MUTEX(&dev->uart_dev);

    return ESP_OK;
}

static esp_err_t exec_cmd(pms1003_t *dev, uint8_t cmd[], size_t delay_ticks)
{
    UART_DEV_TAKE_MUTEX(&dev->uart_dev);
    UART_DEV_CHECK(&dev->uart_dev, send_cmd_nolock(dev, cmd));
    if (delay_ticks)
        vTaskDelay(delay_ticks);
    UART_DEV_GIVE_MUTEX(&dev->uart_dev);

    return ESP_OK;
}

static inline bool is_measuring(pms1003_t *dev)
{
    // not running if measurement is not started
    if (!dev->meas_started)
      return false;

    // not running if time elapsed is greater than duration
    uint64_t elapsed = esp_timer_get_time() - dev->meas_start_time;
    return elapsed < dev->sen.min_period_us;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t pms1003_init_desc(pms1003_t *dev, uart_port_t port, gpio_num_t tx_gpio, gpio_num_t rx_gpio, gpio_num_t rts_io_num, gpio_num_t cts_io_num, int tx_buffer_size, int rx_buffer_size, int queue_size, int intr_alloc_flags, int baud_rate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits, uart_hw_flowcontrol_t flow_ctrl, uint8_t rx_flow_ctrl_thresh, uart_sclk_t source_clk, uint16_t sen_id)
{
    CHECK_ARG(dev);

    dev->uart_dev.port = port;
    dev->uart_dev.tx_io_num = tx_gpio;
    dev->uart_dev.rx_io_num = rx_gpio;
    dev->uart_dev.rts_io_num = rts_io_num;
    dev->uart_dev.cts_io_num = cts_io_num;
    dev->uart_dev.tx_buffer_size = tx_buffer_size;
    dev->uart_dev.rx_buffer_size = rx_buffer_size;
    dev->uart_dev.queue_size = queue_size;
    // dev->uart_dev.queue = queue;
    dev->uart_dev.intr_alloc_flags = intr_alloc_flags;
    dev->uart_dev.cfg.baud_rate = baud_rate;
    dev->uart_dev.cfg.data_bits = data_bits;
    dev->uart_dev.cfg.parity = parity;
    dev->uart_dev.cfg.stop_bits = stop_bits;
    dev->uart_dev.cfg.flow_ctrl = flow_ctrl;
    dev->uart_dev.cfg.source_clk = source_clk;

    memset(&dev->sen, 0, sizeof(sensor_t));
    sensor_init(&dev->sen,12);
    strncpy(dev->sen.name, "PMS1003\0", 8);
    dev->sen.lib_id = SEN_PMS1003_LIB_ID;
    dev->sen.sen_id = sen_id;
    dev->sen.version = 1;
    dev->sen.com_type = SEN_COM_TYPE_DIGITAL_COM;
    dev->sen.min_period_us = 0;
    dev->sen.delay_s_ms = 0;
    dev->sen.out_nr = 12;
    dev->sen.sen_trigger_type = SEN_OUT_TRIGGER_TYPE_TIME;
    dev->sen.period_ms=11001;

    dev->sen.sen_status.fail_cnt = 0;
    dev->sen.sen_status.fail_time = 0;
    dev->sen.sen_status.initialized = false;
    dev->sen.sen_status.delay_m_us = 0;

    dev->sen.sen_outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].out_id=PMS1003_OUT_PM1_0_CON_UNIT_ID;
    dev->sen.sen_outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PM1_0_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].out_id=PMS1003_OUT_PM2_5_CON_UNIT_ID;
    dev->sen.sen_outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PM2_5_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_id=PMS1003_OUT_PM10_CON_UNIT_ID;
    dev->sen.sen_outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_id=PMS1003_OUT_PM10_CON_UNIT_ID;
    dev->sen.sen_outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PM10_CON_UNIT_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].out_id=PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID;
    dev->sen.sen_outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PM1_0_CON_UNIT_ATMOSPHE_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].out_id=PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID;
    dev->sen.sen_outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PM2_5_CON_UNIT_ATMOSPHE_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].out_id=PMS1003_OUT_CON_UNIT_ATMOSPHE_ID;
    dev->sen.sen_outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_CON_UNIT_ATMOSPHE_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_0_3_UM_ID;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_0_3_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_0_5_UM_ID;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_0_5_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_1_0_UM_ID;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_1_0_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_2_5_UM_ID;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_2_5_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_5_0_UM_ID;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_5_0_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].out_id=PMS1003_OUT_PARTICLE_NR_10_UM_ID;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].out_type = SEN_TYPE_DUST;
    dev->sen.sen_outs[PMS1003_OUT_PARTICLE_NR_10_UM_ID].out_val_type=SEN_OUT_VAL_TYPE_UINT16;

    return uart_dev_create_mutex(&dev->uart_dev);
}

esp_err_t pms1003_free_desc(pms1003_t *dev)
{
    CHECK_ARG(dev);

    return uart_dev_delete_mutex(&dev->uart_dev);
}

esp_err_t pms1003_init(pms1003_t *dev)
{
    CHECK_ARG(dev);

    return pms1003_reset(dev);
}

esp_err_t pms1003_reset(pms1003_t *dev)
{
    dev->meas_start_time = 0;
    dev->meas_started = false;

    // CHECK(send_cmd(dev, CMD_RESET));
    vTaskDelay(1);

    return ESP_OK;
}

esp_err_t pms1003_measure(pms1003_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    pms1003_raw_data_t raw;
    uint8_t cmd[7];
    if(get_meas_cmd(cmd) != ESP_OK) return ESP_FAIL;
    CHECK(exec_cmd(dev,cmd, 0));

    return pms1003_compute_values(&raw, temperature, humidity);
}

esp_err_t pms1003_start_measurement(pms1003_t *dev)
{
    CHECK_ARG(dev);

    if (is_measuring(dev))
    {
        ESP_LOGE(TAG, "Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t cmd[7];
    uint16_t check = 0;
    cmd[0] = START_BYTE_1;
    cmd[1] = START_BYTE_2;
    cmd[2] = CMD_READ_PASSIVE_MODE;
    cmd[3] = 0;
    cmd[4] = 0;
    check = cmd[0]+cmd[1]+cmd[2]+cmd[3]+cmd[4];
    cmd[5] = check>>8;
    cmd[6] = (check<<8)>>8;
    ESP_LOGD(TAG, "Sum: %u check_MSB: %u check_LSB: %u", check,cmd[5],cmd[6]);
    UART_DEV_TAKE_MUTEX(&dev->uart_dev);
    dev->meas_start_time = esp_timer_get_time();
    dev->meas_started = true;
    const int txBytes = uart_write_bytes(dev->uart_dev.port, cmd, 7);
    UART_DEV_GIVE_MUTEX(&dev->uart_dev);
    ESP_LOGI(TAG, "Wrote %d bytes", txBytes);
    if(txBytes<7) return ESP_FAIL;

    return ESP_OK;
}

size_t pms1003_get_measurement_duration(pms1003_t *dev)
{
    if (!dev) return 0;

    size_t res = pdMS_TO_TICKS(dev->sen.min_period_us/1000);
    return res == 0 ? 1 : res;
}

esp_err_t pms1003_get_raw_data(pms1003_t *dev, pms1003_raw_data_t *raw)
{
    CHECK_ARG(dev);
    uint64_t timestamp;
    struct timeval tv;
    if (is_measuring(dev))
    {
        ESP_LOGE(TAG, "Measurement is still running");
        return ESP_ERR_INVALID_STATE;
    }
    timestamp = tv.tv_sec * 1000000LL + tv.tv_usec;
    // dev->sen.sen_outs[BME680_OUT_TEMP_ID].measurement_raw = raw_data->temperature;
    // dev->sen.sen_outs[BME680_OUT_TEMP_ID].timestamp = timestamp;
    dev->meas_started = false;
    return read_res(dev, raw);
}

esp_err_t pms1003_compute_values(pms1003_raw_data_t *raw_data, float *temperature, float *humidity)
{
    // CHECK_ARG(raw_data && (temperature || humidity));
    //
    // if (temperature)
    //     *temperature = ((uint16_t)raw_data[0] << 8 | raw_data[1]) * 175.0 / 65535.0 - 45.0;
    //
    // if (humidity)
    //     *humidity = ((uint16_t)raw_data[3] << 8 | raw_data[4]) * 125.0 / 65535.0 - 6.0;

    return ESP_OK;
}

esp_err_t pms1003_get_results(pms1003_t *dev, float *temperature, float *humidity)
{
    pms1003_raw_data_t raw;
    CHECK(pms1003_get_raw_data(dev, &raw));

    return pms1003_compute_values(&raw, temperature, humidity);
}
