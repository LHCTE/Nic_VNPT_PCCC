/**
  ******************************************************************************
  * @file           : modbus _protocol.h
  * @brief          : Header for modbus_protocol.c file  
  *                   This file contains the common defines of the application.
  * @author         : Nichietsu System Development
  ******************************************************************************
  * @note           : None
  ******************************************************************************
  */
#ifndef _MODBUS_RTU_H_
#define _MODBUS_RTU_H_

/* Private includes ----------------------------------------------------------*/
#include "main.h"  

/* Private function prototypes -----------------------------------------------*/
uint16_t ModbusRTU_Calc_CRC16(uint8_t *input, uint8_t size); 
uint16_t ModbusRTU_Check_RecvData(uint8_t *input, uint16_t size, uint16_t timeout); 

#endif /* _MODBUS_RTU_H_ */

/************************ (C) COPYRIGHT Nichietsu System Development *****END OF FILE****/
