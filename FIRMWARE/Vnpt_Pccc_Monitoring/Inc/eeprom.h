/**
  ******************************************************************************
  * @file           : eeprom.h
  * @brief          : Header for eeprom.c file  
  *                   This file contains the common defines of the application.
  * @author         : NgNhatHungThinh & Nichietsu System Development
  ******************************************************************************
  * @note           : Use STM32 flash for Eeprom 
  ******************************************************************************
  */
#ifndef _EEPROM_H_
#define _EEPROM_H_

/* Private includes ----------------------------------------------------------*/
#include "main.h"  

/* Private typedef -----------------------------------------------------------*/
typedef enum { FLASH_ERROR = 0, FLASH_OK = !FLASH_ERROR } Status_FlashTypeDef; 
typedef enum { Position_1 = 0, Position_2 = 8 } Position_FlashTypeDef; 

/* Private function prototypes -----------------------------------------------*/
uint32_t Eeprom_Read_Data(uint32_t addr, Position_FlashTypeDef pos); 
uint32_t Eeprom_ReadModbus_Rtu(uint32_t addr); 
Status_FlashTypeDef Eeprom_WriteStr_Hexadecimal(uint32_t addr, uint64_t *buf, uint16_t timeout); 
Status_FlashTypeDef Eeprom_WriteModbus_Rtu(uint32_t addr, Position_FlashTypeDef pos, uint64_t *buf, uint16_t timeout); 
Status_FlashTypeDef Eeprom_Write_Data(uint32_t addr, Position_FlashTypeDef pos, uint32_t val, uint16_t timeout); 
void Eeprom_ClearPage(uint32_t page); 
void Eeprom_ReadStr_Hexadecimal(uint32_t addr, Position_FlashTypeDef pos, uint32_t *input, uint16_t input_size, uint8_t *output, uint16_t output_size); 

#endif /* _EEPROM_H_ */ 

/************************ (C) COPYRIGHT Nichietsu System Development *****END OF FILE****/
