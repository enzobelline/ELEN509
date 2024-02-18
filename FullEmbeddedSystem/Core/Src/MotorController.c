#include "MotorController.h"

void setMotorSpeed(TIM_HandleTypeDef* htim, uint16_t BLI_Pulse, uint16_t ALI_Pulse ){
  
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  // Channel 2 is ALI
  // Channel 3 is BLI
  HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
  
  // ALI Pulse
  sConfigOC.Pulse = ALI_Pulse;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  
  // BLI PUlse
  sConfigOC.Pulse = BLI_Pulse;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
  
}

void motorInit(TIM_HandleTypeDef* htim){
  HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);
}

void stopMotor(TIM_HandleTypeDef* htim){
  HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3);
}