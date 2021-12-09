/*
 * Copyright (c) 2019-2020 Ecoplanos: Automação e Sistemas Integrados, Lda.
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

#include <sensor_handler.h>

#if CONFIG_CIRC_BUFF_8_T
typedef int8_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_U8_T
typedef uint8_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_16_T
typedef int16_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_U16_T
typedef uint16_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_32_T
typedef int32_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_U32_T
typedef uint32_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_64_T
typedef int64_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_U64_T
typedef uint64_t circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_FLOAT_T
typedef float circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_S_T
#if CONFIG_CIRC_BUFF_S_STATIC
typedef char[CONFIG_CIRC_BUFF_S_STATIC_SIZE] circ_buff_data_type_t;
#else
typedef char *circ_buff_data_type_t;
#endif
#elif CONFIG_CIRC_BUFF_V_T
typedef void circ_buff_data_type_t;
#elif CONFIG_CIRC_BUFF_SENSOR_T
typedef struct{
  uint8_t id;                 //sensor identifier
  sen_out_t *outs;
  // sen_status_t status;
  uint64_t timestamp;         //time when sensor data was sampled
}circ_buff_data_type_t;
// typedef sensor_t circ_buff_data_type_t;

#endif

typedef struct {
  uint8_t buffer_full;
  uint8_t buffer_empty;
  circ_buff_data_type_t* head_idx;
  circ_buff_data_type_t* tail_idx;
  circ_buff_data_type_t* data_buffer;
  // uint16_t data_buffer[CAL_MEASUREMENT_BUFF_SIZE];
}circ_buffer_t;

void initBuffer(uint8_t buff_data_type, uint16_t buff_size);
void deInitBuffer(void);
uint16_t bufferElementsCount(void);
uint8_t bufferPush(circ_buff_data_type_t data);
uint8_t bufferPop(circ_buff_data_type_t *data);
