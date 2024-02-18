#include "Project.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_tim.h"
#include "main.h"
#include <stdint.h>
#include <stddef.h>

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H


void setMotorSpeed(TIM_HandleTypeDef* htim, uint16_t BLI_Pulse, uint16_t ALI_Pulse );
void motorInit(TIM_HandleTypeDef* htim);
void stopMotor(TIM_HandleTypeDef* htim);

#endif