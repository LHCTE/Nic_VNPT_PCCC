#include "eeprom.h"
#include <stdio.h> 
#include <string.h>

uint32_t Eeprom_Read_Data(uint32_t addr, Position_FlashTypeDef pos)
{
	uint32_t data = *(__IO uint32_t *)(addr + pos);
	return data;
}

uint32_t Eeprom_ReadModbus_Rtu(uint32_t addr)
{
	uint32_t data = *(__IO uint32_t *)(addr);
	return data;
}

Status_FlashTypeDef Eeprom_WriteStr_Hexadecimal(uint32_t addr, uint64_t *buf, uint16_t timeout)
{
	uint16_t arr_pos = 0, addr_next = 0; 

	HAL_FLASH_Unlock();

	while(buf[arr_pos * 2] != '\0')
	{
		if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, Position_1 + addr + addr_next, buf[arr_pos * 2]) == HAL_OK && HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, Position_2 + addr + addr_next, buf[(arr_pos * 2) + 1]) == HAL_OK)
		{
			addr_next += 16; 
			arr_pos += 1; 
		}
		else 	
		{
			HAL_Delay(1); 
			timeout--; 

			if(timeout == 0) 
			{
				HAL_FLASH_Lock();
				return FLASH_ERROR; 
			}
		}
	}
	
	HAL_FLASH_Lock();
	return FLASH_OK; 
}
  
Status_FlashTypeDef Eeprom_WriteModbus_Rtu(uint32_t addr, Position_FlashTypeDef pos, uint64_t *buf, uint16_t timeout) 
{
	while(1)
	{
		HAL_FLASH_Unlock();

		switch(pos)
		{
			case Position_1:
				if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, addr + pos, buf[0] | buf[1] << 32) == HAL_OK)
				{
					HAL_FLASH_Lock();
					return FLASH_OK; 
				}
				
				HAL_Delay(1); 
				timeout--; 
				if(timeout == 0) 
				{
					HAL_FLASH_Lock();
					return FLASH_ERROR; 
				}
				break;

			case Position_2:
				if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, addr + pos, buf[0] | buf[1] << 32) == HAL_OK)
				{
					HAL_FLASH_Lock();
					return FLASH_OK; 
				}
				
				HAL_Delay(1); 
				timeout--; 
				if(timeout == 0) 
				{
					HAL_FLASH_Lock();
					return FLASH_ERROR;
				}
				break; 
		}
	}
}

Status_FlashTypeDef Eeprom_Write_Data(uint32_t addr, Position_FlashTypeDef pos, uint32_t val, uint16_t timeout)
{
	while(1)
	{
		HAL_FLASH_Unlock();
		if(HAL_FLASH_Program(TYPEPROGRAM_DOUBLEWORD, addr + pos, val) == HAL_OK) 
		{
			HAL_FLASH_Lock();
			return FLASH_OK;
		}
		
		HAL_Delay(1); 
		timeout--; 
		if(timeout == 0) 
		{
			HAL_FLASH_Lock();
			return FLASH_ERROR; 
		}
	}
} 

void Eeprom_ClearPage(uint32_t page)
{
	HAL_FLASH_Unlock();
	
	FLASH_WaitForLastOperation(1000);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);
	FLASH_PageErase(FLASH_BANK_1, page); 
	FLASH_WaitForLastOperation(1000);
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER); 
  
  /*FLASH_WaitForLastOperation(1000);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR);
	FLASH_PageErase(0, page); 
	FLASH_WaitForLastOperation(1000);
	CLEAR_BIT(FLASH->CR, FLASH_CR_PER); */ 
	
	HAL_FLASH_Lock(); 
}

void Eeprom_ReadStr_Hexadecimal(uint32_t addr, Position_FlashTypeDef pos, uint32_t *input, uint16_t input_size, uint8_t *output, uint16_t output_size)
{
	uint8_t *ptr = 0; 
	uint32_t temp[input_size]; 
	 
	for(uint8_t i = 0; i < input_size; i++)
	{
		input[i] = Eeprom_Read_Data(addr, pos); 
		pos += 4; 
	}

	strcpy((char*)temp, (char*)input);
	ptr = (uint8_t*)&temp; 
	for(uint16_t i = 0; i < output_size; i++) output[i] = ptr[i];
}
