#include "Buttons.h"

keyCode getKeyCode(void){
  if(!HAL_GPIO_ReadPin(BUT_L_GPIO_Port, BUT_L_Pin)){
    return BUT_L;
  } else if(!HAL_GPIO_ReadPin(BUT_R_GPIO_Port, BUT_R_Pin)) {
    return BUT_R;
  } else if(!HAL_GPIO_ReadPin(BUT_ENC_GPIO_Port, BUT_ENC_Pin)) {
    return BUT_ENC;
  }
  
  return BUT_NULL;
}