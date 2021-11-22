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

#include <stdint.h>

#define POT_LOG_TAG "MCP42X1"

#define POT_RS  (double)((double)POT_RAB_VAL/(double)POT_RESISTORS)

#define POT_WIPPER_MAX POT_RESISTORS
#define POT_WIPPER_MIN 0

#define POT_SHDN 21  // GPIO21/pin33
#define POT_SHDN_PIN_SEL  (1ULL<<POT_SHDN)
#define POT_OFF   0
#define POT_ON    1

#define POT_SCK   17  // SPI SCK -> GPIO17/pin28
#define POT_CS    18  // SPI CS -> GPIO18/pin30
#define POT_SDO   19  // SPI SDI -> GPIO19/pin31
#define POT_SDI   16  // SPI SDO -> GPIO16/pin27

#define POT_W0_ADDR     0x00
#define POT_W1_ADDR     0x01
#define POT_TCON_ADDR   0x04

#define POT_TCON_R1HW   0x80
#define POT_TCON_R1A    0x40
#define POT_TCON_R1W    0x20
#define POT_TCON_R1B    0x10
#define POT_TCON_R0HW   0x08
#define POT_TCON_R0A    0x04
#define POT_TCON_R0W    0x02
#define POT_TCON_R0B    0x01

#define POT_STATUS_ADDR 0x05
#define POT_STATUS_SHDN 0b10

#define POT_CMDERR 0x200
#define POT_WRITE_DATA_CMD  0b0000
#define POT_INCR_CMD        0b0100
#define POT_DECR_CMD        0b1000
#define POT_READ_DATA_CMD   0b1100

#define POT_COMMAND_BITS 4
#define POT_ADDR_BITS 4
#define POT_DATA_UNUSED_BITS 2

//TODO: this values are made up! xD
#define POT_SPI_TRANSF_SZ 1
#define POT_SPI_FREQ 100000
#define POT_RESISTORS 10
#define POT_RAB_VAL 100

#define POT_COMMAND_CONSTRUCT(addr, cmd) ((addr<<4)|cmd)
#define POT_COMMAND_CHECK(resp) ((resp&POT_CMDERR)>>9)
/*#define POT_TCON_DATA (POT_TCON_R1HW|POT_TCON_R1A|\
POT_TCON_R1W|POT_TCON_R1B|POT_TCON_R0HW|\
POT_TCON_R0A|POT_TCON_R0W|POT_TCON_R0B)*/
/*#define POT_TCON_DATA (POT_TCON_R1A|\
POT_TCON_R1W|POT_TCON_R1B|\
POT_TCON_R0A|POT_TCON_R0W|POT_TCON_R0B)*/
/*#define POT_TCON_DATA (POT_TCON_R1HW|POT_TCON_R1A|\
POT_TCON_R1W|POT_TCON_R0HW|\
POT_TCON_R0A|POT_TCON_R0W)*/
// #define POT_TCON_DATA (POT_TCON_R0B|POT_TCON_R1B|POT_TCON_R0W|POT_TCON_R1W)
// #define POT_TCON_DATA (POT_TCON_R0A|POT_TCON_R1A|POT_TCON_R0W|POT_TCON_R1W)
#define POT_TCON_DATA (POT_TCON_R1A|POT_TCON_R1W|POT_TCON_R0A|POT_TCON_R0W|POT_TCON_R1HW|POT_TCON_R0HW)

void pot_shdn(uint8_t on_off);
void pot_init(void);
void pot_set_r0aw(double res_value);
void pot_set_r0bw(double res_value);
void pot_set_r1aw(double res_value);
void pot_set_r1bw(double res_value);
void pot_set_w0(uint8_t wipper0);
void pot_set_w1(uint8_t wipper1);
void pot_set_w01(uint8_t wipper0, uint8_t wipper1);
