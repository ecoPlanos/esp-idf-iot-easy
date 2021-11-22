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


#ifndef __PASSIVE_COMPONENTS_HAND_H__
#define __PASSIVE_COMPONENTS_HAND_H__

// #include <driver/gpio.h>
#include <esp_err.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
#include <stdbool.h>
// #include <sys/time.h>

#define PASSIVE_COMPONENTS_LIB_VERSION_MAJOR 1
#define PASSIVE_COMPONENTS_LIB_VERSION_MINOR 0
#define PASSIVE_COMPONENTS_LIB_VERSION_PATCH 0
#define PASSIVE_COMPONENTS_LIB_VERSION  (PASSIVE_COMPONENTS_LIB_VERSION_MAJOR << 16)|(PASSIVE_COMPONENTS_LIB_VERSION_MINOR << 8)|PASSIVE_COMPONENTS_LIB_VERSION_PATCH

#define PASSIVE_COMPONENTS_MAX_REF_NAME_LEN (CONFIG_PASSIVE_COMPONENTS_MAX_REF_NAME_LEN + 1)

typedef enum {
  PASSIVE_COMPONENTS_SI_PERFIX_YOTTA = 1,
  PASSIVE_COMPONENTS_SI_PERFIX_ZETTA,
  PASSIVE_COMPONENTS_SI_PERFIX_EXA,
  PASSIVE_COMPONENTS_SI_PERFIX_PETA,
  PASSIVE_COMPONENTS_SI_PERFIX_TERA,
  PASSIVE_COMPONENTS_SI_PERFIX_GIGA,
  PASSIVE_COMPONENTS_SI_PERFIX_MEGA,
  PASSIVE_COMPONENTS_SI_PERFIX_KILO,
  PASSIVE_COMPONENTS_SI_PERFIX_HECTO,
  PASSIVE_COMPONENTS_SI_PERFIX_DECA,
  PASSIVE_COMPONENTS_SI_PERFIX_DECI,
  PASSIVE_COMPONENTS_SI_PERFIX_CENTI,
  PASSIVE_COMPONENTS_SI_PERFIX_MILLI,
  PASSIVE_COMPONENTS_SI_PERFIX_MICRO,
  PASSIVE_COMPONENTS_SI_PERFIX_PICO,
  PASSIVE_COMPONENTS_SI_PERFIX_FEMTO,
  PASSIVE_COMPONENTS_SI_PERFIX_ATTO,
  PASSIVE_COMPONENTS_SI_PERFIX_ZEPTO,
  PASSIVE_COMPONENTS_SI_PERFIX_YOCTO
} passive_units_si_perfix_t;

typedef enum {
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_YOTTA = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_ZETTA = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_EXA = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_PETA = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_TERA = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_GIGA = 1000000000,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_MEGA = 1000000,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_KILO = 1000,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_HECTO = 100,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_DECA = 10,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_DECI = 10,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_CENTI = 100,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_MILLI = 1000,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_MICRO = 1000000,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_PICO = 1000000000,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_FEMTO = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_ATTO = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_ZEPTO = 0,
  PASSIVE_COMPONENTS_SI_PERFIX_FACTOR_YOCTO = 0
} passive_units_si_perfix__t;

typedef enum {
  PASSIVE_COMPONENTS_RESISTOR = 1,
  PASSIVE_COMPONENTS_CAPACITOR,
  PASSIVE_COMPONENTS_INDUCTOR,
  PASSIVE_COMPONENTS_CRISTAL,
  PASSIVE_COMPONENTS_OSCILATOR,
  PASSIVE_COMPONENTS_RESSONATOR,
  PASSIVE_COMPONENTS_COMMON_MODE_CHOKE,
  PASSIVE_COMPONENTS_FERRITE
} passive_component_type_t;

typedef enum {
  PASSIVE_COMPONENTS_CONN_NONE = 1,
  PASSIVE_COMPONENTS_CONN_SERIES,
  PASSIVE_COMPONENTS_CONN_PARALLEL
} passive_component_conn_type_t;

//Passive component structure
typedef struct{
  passive_component_type_t type;
  char design_ref_name[PASSIVE_COMPONENTS_MAX_REF_NAME_LEN];  //Correspondent component design reference name
  uint32_t value;                                             //tolerance in %
  passive_units_si_perfix_t val_unit_per;                     //SI unit perfix of value
  float tolerance;                                            //tolerance in %
  float temp_coef;                                            //temperature coefficient in parts per million (ppm)
  struct passive_t *connected;                                //series/parallel component connection_*
  passive_component_conn_type_t conn;                         //how it is connectedo to child
}passive_t;


// void passive_calc_series(passive_t* sensor, uint8_t outs_nr);
// void passive_calc_parallel(passive_t* sensor, uint8_t outs_nr);
esp_err_t passive_component_init(passive_t* component, passive_component_type_t component_type, const char* name, uint32_t value, passive_units_si_perfix_t val_unit_per, float tolerance, float temp_coef);
#endif /* __PASSIVE_COMPONENTS_HAND_H__ */
