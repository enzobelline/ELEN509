#include "Project.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#include "stm32f3xx_hal_def.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>

#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#define RTC_ADDRESS 0xDE



HAL_StatusTypeDef rtcReadTime(uint8_t* data);
HAL_StatusTypeDef rtcWriteTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);

HAL_StatusTypeDef rtcStart();
HAL_StatusTypeDef rtcVbatEnable();

#endif
