/**
  ******************************************************************************
  * @file           : espressifAT.h
  * @brief          : Header for espressifAT.c file  
  *                   This file contains the common defines of the application.
  * @author         : Nichietsu System Development
  ******************************************************************************
  * @note           : Use AT commands 
  ******************************************************************************
  */
#ifndef _ESPRESSIF_AT_H_
#define _ESPRESSIF_AT_H_

#ifdef ESP32
#define __ESP32__
#else
#define __ESP8266__
#endif 

/* Private includes ----------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define _ms_    100
#define Host_Byte     30 
#define ID_Byte       25
#define Username_Byte 25
#define Password_Byte 25 
#define TopicSub_Byte 30
#define TopicPub_Byte 30

/* Private typedef -----------------------------------------------------------*/
typedef enum { ESP32_ERROR = 0, ESP32_OK = !ESP32_ERROR } ESP32_StatusTypeDef; 
//typedef enum { DISABLE = 0, ENABLE = !DISABLE }ControlTypeDef; 

typedef enum 
{
	ESP_TOUCH_App           = 0, 
	AIRKISS_App             = 1, 
	TOUTCH_and_AIRKISS_App  = 3
}App_HandleTypeDef; 

typedef enum 
{
	NULL_Mode       = 0,
	STA_Mode        = 1, 
	AP_Mode         = 2, 
	AP_and_STA_Mode = 3
}Mode_HandleTypeDef; 

typedef struct
{
  uint8_t sub[TopicSub_Byte];
  uint8_t pub[TopicPub_Byte]; 
}Topic_HandleTypeDef; 

typedef struct
{
  uint8_t host[Host_Byte]; 
	uint16_t port;
  uint8_t id[ID_Byte]; 
	uint8_t user[Username_Byte];
	uint8_t password[Password_Byte];  	
  Topic_HandleTypeDef topic; 
}MQTT_StructuresTypeDef;  

/* Private function prototypes -----------------------------------------------*/
#ifdef __ESP32__
ESP32_StatusTypeDef Esp32_Response(uint8_t *recv_buf, uint8_t *content, uint16_t timeout); 
int8_t RSSI_WiFi(UART_HandleTypeDef huart, uint8_t *recv_buf); 
void Esp32_Check_Version(UART_HandleTypeDef huart); 
void Esp32_Restart(UART_HandleTypeDef huart); 
void Esp32_Echo(UART_HandleTypeDef huart, FunctionalState ctrl); 
void Esp32_WiFiMode(UART_HandleTypeDef huart, Mode_HandleTypeDef mode); 
void Esp32_Start_SmartConfig(UART_HandleTypeDef huart, App_HandleTypeDef app); 
void Esp32_Stop_SmartConfig(UART_HandleTypeDef huart); 
void Esp32_Check_Info_WiFi(UART_HandleTypeDef huart);
void Esp32_Setup_Default(UART_HandleTypeDef huart); 

void Esp32_Mqtt_UserConfig(UART_HandleTypeDef huart, uint8_t *id, uint8_t *username, uint8_t *password); 
void Esp32_Mqtt_Connect_Broker(UART_HandleTypeDef huart, uint8_t *host, uint16_t port); 
void Esp32_Mqtt_Disconnect_Broker(UART_HandleTypeDef huart); 
void Esp32_Mqtt_Subscribe(UART_HandleTypeDef huart, uint8_t *topic); 
void Esp32_Mqtt_Publish(UART_HandleTypeDef huart, uint8_t *recv_buf, uint8_t *topic, uint8_t *msg); 
#endif

#endif /* _ESPRESSIF_AT_H_ */ 

/************************ (C) COPYRIGHT Nichietsu System Development *****END OF FILE****/
