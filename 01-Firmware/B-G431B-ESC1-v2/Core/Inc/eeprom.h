/*
 * eeprom.h
 *
 *  Created on: 13 déc. 2020
 *      Author: Patrick
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include <stdbool.h>
#include "stm32g4xx_hal.h"

bool eeprom_empty();
HAL_StatusTypeDef eeprom_restore(uint8_t *regs, int *regs_lut, uint32_t size, uint32_t size_lut);           // size must be 64 bits aligned
HAL_StatusTypeDef eeprom_store(uint8_t const *regs, int const *regs_lut, uint32_t size, uint32_t size_lut); // size must be 64 bits aligned

uint8_t const *eeprom_base_address();

#endif /* INC_EEPROM_H_ */
