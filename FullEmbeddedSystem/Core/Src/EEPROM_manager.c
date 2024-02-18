#include "EEPROM_manager.h"

extern I2C_HandleTypeDef hi2c1;


void EEPROM_Write(uint8_t *data){
  HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, data, 16, 1000);
  HAL_Delay(5);
  HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, 0x30, I2C_MEMADD_SIZE_8BIT, &data[16], 8, 1000);
}

void EEPROM_Read(uint8_t *data){
  HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, data, 24, 1000);
}
