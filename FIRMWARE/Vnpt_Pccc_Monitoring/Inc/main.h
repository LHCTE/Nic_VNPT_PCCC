/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Relay_Pin GPIO_PIN_13
#define Relay_GPIO_Port GPIOC
#define A7670C_SimCom_Tx_Pin GPIO_PIN_2
#define A7670C_SimCom_Tx_GPIO_Port GPIOA
#define A7670C_SimCom_Rx_Pin GPIO_PIN_3
#define A7670C_SimCom_Rx_GPIO_Port GPIOA
#define Checking_Esp32_Pin GPIO_PIN_4
#define Checking_Esp32_GPIO_Port GPIOA
#define PhysRestart_Sim7020_Pin GPIO_PIN_7
#define PhysRestart_Sim7020_GPIO_Port GPIOA
#define ESP32_WiFi_Rx_Pin GPIO_PIN_0
#define ESP32_WiFi_Rx_GPIO_Port GPIOB
#define ESP32_WiFi_Tx_Pin GPIO_PIN_2
#define ESP32_WiFi_Tx_GPIO_Port GPIOB
#define DC_LED_Pin GPIO_PIN_10
#define DC_LED_GPIO_Port GPIOB
#define Battery_LED_Pin GPIO_PIN_11
#define Battery_LED_GPIO_Port GPIOB
#define Test_LED_Pin GPIO_PIN_12
#define Test_LED_GPIO_Port GPIOB
#define SOS_LED_Pin GPIO_PIN_13
#define SOS_LED_GPIO_Port GPIOB
#define RS485_Modbus_Tx_Pin GPIO_PIN_9
#define RS485_Modbus_Tx_GPIO_Port GPIOA
#define RS485_Modbus_Rx_Pin GPIO_PIN_10
#define RS485_Modbus_Rx_GPIO_Port GPIOA
#define Input_Device_Pin GPIO_PIN_3
#define Input_Device_GPIO_Port GPIOB
#define Button_Pin GPIO_PIN_5
#define Button_GPIO_Port GPIOB
#define PhysRestart_Pin GPIO_PIN_7
#define PhysRestart_GPIO_Port GPIOB
#define WiFi_Reset_Pin GPIO_PIN_8
#define WiFi_Reset_GPIO_Port GPIOB
#define IO2_Pin GPIO_PIN_9
#define IO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//...Info device 
#define DeviceID "00032" 
#define Version "Ver 1.0" 
#define Server PRODUCT_SERVER

//...Real Time clock 
#define Setup_Day 12
#define Setup_Month 8
#define Setup_Year 22
#define Setup_Hour 17
#define Setup_Minute 22

//...Byte amount 
#define Send_Byte 2000
#define Receive_Byte 1300
#define IoT_Byte 500
#define Inverter_Byte 500 
#define Filter_Byte 500 
#define Eeprom_Byte 2
#define Eeprom_Host_Byte 7 
#define Eeprom_Username_Byte 2
#define Eeprom_Password_Byte 2
#define Eeprom_TopicSub_Byte 4
#define Eeprom_TopicPub_Byte 5

//...WiFi light status 
#define Check_Connect_WiFi 150 
#define SmartConfig_WiFi 10 
#define Connected_WiFi 0

//...4GLTE light status 
#define Check_Insert_Sim_Card 10 //150 
#define Network_Service_Standby 0 

//...Data light status
#define Data_Processing 10
#define Normal_Mode 150 
#define Config_Mode 5
#define Check_Mode 0

//...Eeprom 
#define User_ConfigInfomation_Page 59
#define Addr_BrokerFlag 0x0801D800
#define Addr_Host 0x0801D810
#define Addr_Port 0x0801D840
#define Addr_ID 0x0801D840
#define Addr_Username 0x0801D850
#define Addr_Password 0x0801D860
#define Addr_TopicSub 0x0801D870
#define Addr_TopicPub 0x0801D880
#define User_ConfigInterval_Page 60
#define Addr_Interval 0x0801E000
#define User_ConfigBaudrate_Page 61
#define Addr_Baudrate 0x0801E800
#define User_ConfigCmd2Cmd_Page 62
#define Addr_C2C 0x0801F000
#define User_ConfigRs485Cmd_Page 63
#define Addr_CmdConfig 0x0801F800
#define Addr_Data_Rs485 0x0801F810
#define Addr_Size_Rs485 0x0801F810

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
