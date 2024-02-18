#include "RTC_manager.h"

extern I2C_HandleTypeDef hi2c1;

// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}


HAL_StatusTypeDef rtcReadTime(uint8_t* data){
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Read(&hi2c1, RTC_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, data, 7, 1000);
  
  for (int i = 0; i < 7; ++i){
    data[i] = bcdToDec(data[i]);
  }
  return status;
}

HAL_StatusTypeDef rtcWriteTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year) {
  uint8_t set_time[7];
  set_time[0] = decToBcd(sec);
  set_time[1] = decToBcd(min);
  set_time[2] = decToBcd(hour);
  set_time[3] = decToBcd(dow);
  set_time[4] = decToBcd(dom);
  set_time[5] = decToBcd(month);
  set_time[6] = decToBcd(year);
  
  return HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, set_time, 7, 1000);
}

HAL_StatusTypeDef rtcStart(){
  uint8_t value = 0x80;
  return HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

HAL_StatusTypeDef rtcVbatEnable(){
  uint8_t value = 0x08;
  return HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS, 0x03, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);

}

//HAL_StatusTypeDef rtcStart(){
//}
//
//HAL_StatusTypeDef rtcSetSQWV(){
//
//}
//
//
//HAL_StatusTypeDef rtcReadRAM(uint8_t* data){
//  return HAL_I2C_Mem_Read(&hi2c1, RTC_I2C_ADDRESS, 0x20, 
//                          I2C_MEMADD_SIZE_8BIT, data, 8, 1000);
//}
//
//HAL_StatusTypeDef rtcWriteRAM(uint8_t* data){
//  return HAL_I2C_Mem_Write(&hi2c1, RTC_I2C_ADDRESS, 0x20, 
//                           I2C_MEMADD_SIZE_8BIT, data, 8, 1000);
//}
