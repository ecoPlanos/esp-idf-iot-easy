/*
 * Copyright (c) 2019 Ecoplanos: Automação e Sistemas Integrados, Lda.
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

#include "mcp42x1.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"

static uint8_t pot_on;
static uint16_t *pot_rx_data;
static esp_err_t ret;
static spi_device_handle_t spi;
static spi_transaction_t spi_trans;

void pot_spi_pre_transfer_callback(spi_transaction_t *t)
{
  ESP_LOGD(POT_LOG_TAG, "pot_spi_pre_transfer_callback");
  int dc=(int)t->user;
}
void pot_spi_post_transfer_callback(spi_transaction_t *t)
{
  ESP_LOGD(POT_LOG_TAG, "pot_spi_post_transfer_callback");
  int dc=(int)t->user;
}

static const spi_bus_config_t buscfg={
  .mosi_io_num=POT_SDI,
  .miso_io_num=POT_SDO,
  .sclk_io_num=POT_SCK,
  .quadwp_io_num=-1,
  .quadhd_io_num=-1,
  .max_transfer_sz=POT_SPI_TRANSF_SZ,
  .flags=SPICOMMON_BUSFLAG_MASTER|SPICOMMON_BUSFLAG_SCLK|SPICOMMON_BUSFLAG_MISO|SPICOMMON_BUSFLAG_MOSI,
};
static const spi_device_interface_config_t devcfg={
  // .command_bits=POT_ADDR_BITS,
  // .address_bits=POT_COMMAND_BITS,
  .command_bits=0,
  .address_bits=0,
  .dummy_bits=0,
  .mode=0,
  .duty_cycle_pos=128,
  // .cs_ena_pretrans=0,   //Amount of SPI bit-cycles the cs should be activated before the transmission (0-16). This only works on half-duplex transactions.
  // .cs_ena_posttrans=0,  //Amount of SPI bit-cycles the cs should stay active after the transmission (0-16)
  .clock_speed_hz=POT_SPI_FREQ,
  // .input_delay_ns=21,   //Maximum data valid time of slave. The time required between SCLK and MISO valid, including the possible clock delay from slave to master. The driver uses this value to give an extra delay before the MISO is ready on the line.
  .input_delay_ns=0,   //Maximum data valid time of slave. The time required between SCLK and MISO valid, including the possible clock delay from slave to master. The driver uses this value to give an extra delay before the MISO is ready on the line.
  .spics_io_num=POT_CS,
  .flags=SPI_DEVICE_NO_DUMMY,
  .queue_size=1,
  // .pre_cb=pot_spi_pre_transfer_callback,
  // .post_cb=pot_spi_post_transfer_callback
};

void pot_shdn(uint8_t on_off)
{
  gpio_set_level(POT_SHDN, on_off);
  pot_on = on_off;
}

void pot_init(void)
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = POT_SHDN_PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;

  gpio_config(&io_conf);
  pot_shdn(POT_OFF);

  ret=spi_bus_initialize(HSPI_HOST, &buscfg, 0);
  ESP_ERROR_CHECK(ret);
  ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  memset(&spi_trans,0,sizeof(spi_transaction_t));
  spi_trans.flags=SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA;
  // spi_trans.flags=SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA|SPI_TRANS_VARIABLE_CMD|SPI_TRANS_VARIABLE_ADDR;
  // spi_trans.cmd=POT_TCON_ADDR;
  // spi_trans.addr=POT_WRITE_DATA_CMD;
  spi_trans.length=16;
  spi_trans.rxlength=16;
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_TCON_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=POT_TCON_DATA;
  spi_trans.rx_buffer=NULL;

  pot_rx_data = spi_trans.rx_data;

  gpio_set_level(POT_SHDN, POT_OFF);

  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "SPI failed to init!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
  else
  {
    spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_TCON_ADDR,POT_READ_DATA_CMD);
    // spi_trans.cmd=POT_TCON_ADDR;
    // spi_trans.addr=POT_READ_DATA_CMD;
    ret=spi_device_transmit(spi, &spi_trans);
    ESP_ERROR_CHECK(ret);
    if(!POT_COMMAND_CHECK(*pot_rx_data))
    {
      ESP_LOGE(POT_LOG_TAG, "SPI failed to init!\tReceived data: %u", *pot_rx_data);
      //TODO
    }
    if(spi_trans.rx_data[1]==POT_TCON_DATA)
      ESP_LOGI(POT_LOG_TAG, "SPI INIT OK!");
    else
    {
      ESP_LOGE(POT_LOG_TAG, "SPI failed to init!\tPOT_TCON_DATA: %u\tReceived data: %u", POT_TCON_DATA, *pot_rx_data);
      // TODO
    }
  }
}

void pot_set_r0aw(double res_value)
{
  // spi_trans.cmd=POT_WRITE_DATA_CMD;
  // spi_trans.addr=POT_W0_ADDR;
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_W0_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=(uint8_t)(((uint32_t)POT_RESISTORS)-((uint32_t)(res_value/POT_RS)));
  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "CMDERR @ pot_set_r0aw()!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
}
void pot_set_r0bw(double res_value)
{
  // spi_trans.cmd=POT_WRITE_DATA_CMD;
  // spi_trans.addr=POT_W0_ADDR;
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_W0_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=(uint8_t)(res_value/POT_RS);
  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "CMDERR @ pot_set_r0bw()!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
}
void pot_set_r1aw(double res_value)
{
  // spi_trans.cmd=POT_WRITE_DATA_CMD;
  // spi_trans.addr=POT_W1_ADDR;
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_W1_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=(uint8_t)(POT_RESISTORS-(res_value/POT_RS));
  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "CMDERR @ pot_set_r1aw()!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
}
void pot_set_r1bw(double res_value)
{
  // spi_trans.cmd=POT_WRITE_DATA_CMD;
  // spi_trans.addr=POT_W1_ADDR;
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_W1_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=(uint8_t)(res_value/POT_RS);
  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "CMDERR @ pot_set_r1bw()!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
}
void pot_set_w0(uint8_t wipper0)
{
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_W0_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=wipper0;
  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "CMDERR @ pot_set_w0()!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
}
void pot_set_w1(uint8_t wipper1)
{
  spi_trans.tx_data[0]=POT_COMMAND_CONSTRUCT(POT_W1_ADDR,POT_WRITE_DATA_CMD);
  spi_trans.tx_data[1]=wipper1;
  ret=spi_device_transmit(spi, &spi_trans);
  ESP_ERROR_CHECK(ret);
  if(!POT_COMMAND_CHECK(*pot_rx_data))
  {
    ESP_LOGE(POT_LOG_TAG, "CMDERR @ pot_set_w1()!\tReceived data: %u", *pot_rx_data);
    //TODO
  }
}
