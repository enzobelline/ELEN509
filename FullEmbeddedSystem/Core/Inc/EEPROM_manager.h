#include "Project.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_def.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#define EEPROM_ADDR 0xA0

void EEPROM_Read (uint8_t *data);
void EEPROM_Write (uint8_t *data);


#endif