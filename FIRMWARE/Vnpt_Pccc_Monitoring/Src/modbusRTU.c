#include "modbusRTU.h"
#include <string.h>

uint16_t ModbusRTU_Calc_CRC16(uint8_t *input, uint8_t size)
{
	//Initialization of crc 16-bit register to 0xFFFF for CCITT 
	uint16_t reg_crc = 0xFFFF; 
	uint8_t crc_high = 0x00, crc_low = 0x00; 
	
	for(uint8_t pos = 0; pos < size - 2; pos++) //-2: Remove 2 last byte 
	{
		reg_crc ^= input[pos]; //XOR byte into least sig. byte of crc
		
		//Loop over each bit
		for(uint8_t j = 0; j < 8; j++) 
		{
			//If the LSB is set
			if((reg_crc & 0x01) == 1) reg_crc = (reg_crc >> 1) ^ 0xA001; //Shift right and XOR 0xA001
			else reg_crc = (reg_crc >> 1) ^ 0x0000; //Just shift right
		}
		
		crc_high = (reg_crc >> 8) & 0xFF; 
		crc_low = reg_crc & 0xFF; 		 
	}    
	if(input[size - 2] == crc_low && input[size - 1] == crc_high) return size; 
	else return 0;
}

uint16_t ModbusRTU_Check_RecvData(uint8_t *input, uint16_t size, uint16_t timeout)
{
	uint16_t total = 0;
	while(1) 
	{
		for(uint16_t i = 0; i < size; i++) total += input[i]; //...Sum Hex 
		if(total > 0) return total; //If have valued
		
		HAL_Delay(1); 
		timeout--; //...Timer
		if(timeout == 0) return 0; //End of time
	}
}
