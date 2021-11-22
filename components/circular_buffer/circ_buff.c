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

#include "circ_buff.h"
#define TAG "CIRC_BUFF"

static circ_buffer_t w_buffer;
static uint8_t _buff_data_type;
static uint16_t _buff_size;
// void initBuffer(uint8_t buff_data_type, uint16_t buff_size)
// {
// 	_buff_size=buff_size;
// 	if((buff_data_type < CIRC_BUFF8_T) || (buff_data_type > CIRC_BUFF64_T))
// 	{
// 		ESP_LOGE(CIRC_BUFF_TAG, "Invalid data type! changing to 64bit...");
// 		_buff_data_type = CIRC_BUFF64_T;
// 	}
// 	else
// 	{
// 		_buff_data_type = buff_data_type;
// 	}
// 	w_buffer.data_buffer = malloc(_buff_size);
//   w_buffer.head_idx = w_buffer.data_buffer;
// 	switch(_buff_data_type)
// 	{
// 		case CIRC_BUFF8_T:
// 			w_buffer.tail_idx = (void *)((uint8_t *)(w_buffer.data_buffer)+_buff_size-1));
// 		break;
// 		case CIRC_BUFF16_T:
// 			w_buffer.tail_idx = (void *)((uint16_t *)(w_buffer.data_buffer)+_buff_size-1));
// 		break;
// 		case CIRC_BUFF32_T:
// 			w_buffer.tail_idx = (void *)((uint32_t *)(w_buffer.data_buffer)+_buff_size-1));
// 		break;
// 		case CIRC_BUFF64_T:
// 			w_buffer.tail_idx = (void *)((uint64_t *)(w_buffer.data_buffer)+_buff_size-1));
// 		break;
// 	}
//
//   w_buffer.buffer_empty = 1;
//   w_buffer.buffer_full = 0;
// }
//
// void deInitBuffer(void)
// {
// 	// free(w_buffer.data_buffer);
// }
//
// static void calc_next(void **next_)
// {
// 	switch(_buff_data_type)
// 	{
// 		case CIRC_BUFF8_T:
// 			uint8_t **next = (uint8_t **)next_;
// 		break;
// 		case CIRC_BUFF16_T:
// 			uint16_t **next = (uint16_t **)next_;
// 		break;
// 		case CIRC_BUFF32_T:
// 			uint32_t **next = (uint32_t **)next_;
// 		break;
// 		case CIRC_BUFF64_T:
// 			uint64_t **next = (uint64_t **)next_;
// 		break;
// 	}
//   if(*next-w_buffer.data_buffer == _buff_size-1)
//   {
//     *next = w_buffer.data_buffer;
//   }
//   else
//   {
//     (*next)++;
//   }
// }
//
// uint8_t bufferPush(void *data_)
// {
// 		switch(_buff_data_type)
// 		{
// 			case CIRC_BUFF8_T:
// 				uint8_t *next = (uint8_t *) w_buffer.head_idx;
// 				uint8_t *data = (uint8_t *) data_;
// 				calc_next(&next);
// 				if (next == (uint8_t *) w_buffer.tail_idx) // check if circular buffer is full
// 				{
// 					ESP_LOGD(CIRC_BUFF_TAG,"buffer is full");
//
// 					*next = *data; // Load data and then move
//
// 					w_buffer.buffer_full = 1;
// 					return 0;
// 				}
// 				else
// 		    {
// 		      w_buffer.buffer_full = 0;
//
// 		      *next = *data; // Load data and then move
// 		      w_buffer.head_idx = next;	// head to next data offset.
//
// 		      return 1;  // return success to indicate successful push.
// 		    }
//
// 			break;
// 			case CIRC_BUFF16_T:
// 				uint16_t *next = w_buffer.head_idx;
//
// 			break;
// 			case CIRC_BUFF32_T:
// 				uint32_t *next = w_buffer.head_idx;
//
// 			break;
// 			case CIRC_BUFF64_T:
// 				uint64_t *next = w_buffer.head_idx;
//
// 			break;
// 		}
// 		return 1;
// }
//
// uint8_t bufferPop(void *data)
// {
//     uint16_t *next = w_buffer.tail_idx;
//     calc_next(&next);
//     if (next == w_buffer.head_idx) // check if circular buffer is empty
//     {
//       ESP_LOGD(CIRC_BUFF_TAG,"bufferPop() - Data buffer wasn't initialized yet!");
//
//       w_buffer.buffer_empty = 1;
//       return 0;          // and return with an error
//     }
//     else
//     {
//       w_buffer.buffer_empty = 0;
//
//       w_buffer.tail_idx = next;     // tail to next data offset.
//       *data = *(w_buffer.tail_idx); // Read data
//
//       return 1;
//     }
// }
// uint16_t bufferElementsCount(void)
// {
//   uint16_t *next = w_buffer.tail_idx;
//   calc_next(&next);
//   if (next == w_buffer.head_idx) // check if circular buffer is empty
//   {
//     ESP_LOGD(CIRC_BUFF_TAG,"bufferElementsCount() - Data buffer wasn't initialized yet!");
//     return 0;
//   }
//   else if(w_buffer.tail_idx > w_buffer.head_idx)
//   {
//     ESP_LOGD(CIRC_BUFF_TAG,"(head < tail)Elements remaining on buffer: %u", (uint32_t)((w_buffer.head_idx-w_buffer.data_buffer)+((w_buffer.data_buffer+_buff_size-1)-w_buffer.tail_idx)));
//     return ((w_buffer.head_idx-w_buffer.data_buffer)+((w_buffer.data_buffer+_buff_size-1)-w_buffer.tail_idx-1));
//   }
//   else if(w_buffer.head_idx > w_buffer.tail_idx)
//   {
//     ESP_LOGD(CIRC_BUFF_TAG,"(head > tail)Elements remaining on buffer: %u", (uint32_t)(w_buffer.head_idx-w_buffer.tail_idx-1));
//     return w_buffer.head_idx-w_buffer.tail_idx - 1;
//   }
//   else
//   {
//     ESP_LOGE(CIRC_BUFF_TAG,"Impossible!");
//     return 0;
//   }
// 	return 0;
// }
