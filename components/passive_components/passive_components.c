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
// #include <stdlib.h>
#include <esp_log.h>
#include "passive_components.h"

#define TAG "passive_components"

esp_err_t passive_component_init(passive_t* component, passive_component_type_t component_type, const char* name, uint32_t value, passive_units_si_perfix_t val_unit_per, float tolerance, float temp_coef) {
  size_t len = strlen(name);

  if(len>=PASSIVE_COMPONENTS_MAX_REF_NAME_LEN) {
    ESP_LOGE(TAG, "Component reference name is to long. max: %u. Check menuconfig to increase length.",PASSIVE_COMPONENTS_MAX_REF_NAME_LEN-1);
    return ESP_ERR_INVALID_ARG;
  }

  memset(component,0, sizeof(passive_t));
  component->type = component_type;
  memset(component->design_ref_name,'\0', PASSIVE_COMPONENTS_MAX_REF_NAME_LEN);
  memcpy(component->design_ref_name, name, len);
  component->value = value;
  component->val_unit_per = val_unit_per;
  component->tolerance = tolerance;
  component->temp_coef = temp_coef;
  component->conn = PASSIVE_COMPONENTS_CONN_NONE;

  return ESP_OK;
}
