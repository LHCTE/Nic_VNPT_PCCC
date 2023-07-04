#include "espressifAT.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 

ESP32_StatusTypeDef Esp32_Response(uint8_t *recv_buf, uint8_t *content, uint16_t timeout)
{
	while(1)
	{
		uint8_t *_pStr = (uint8_t*)strstr((char*)recv_buf, (char*)content); 
		if(_pStr != 0) return ESP32_OK; 
		
		HAL_Delay(1);
		timeout--; 
		if(timeout == 0) return ESP32_ERROR; 
	}
}

int8_t RSSI_WiFi(UART_HandleTypeDef huart, uint8_t *recv_buf)
{
	uint8_t *pStr_Analysis = 0; 
	int8_t val = 0; 
	
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWJAP?\r\n", strlen("AT+CWJAP?\r\n"), 100); 
	HAL_Delay(_ms_); 
	pStr_Analysis = (uint8_t*)strstr((char*)recv_buf, "-"); 
	val = atoi((char*)pStr_Analysis);
	return val; 
}

void Esp32_Check_Version(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+GMR\r\n", strlen("AT+GMR\r\n"), 100); 
	HAL_Delay(_ms_); 
}

void Esp32_Restart(UART_HandleTypeDef huart)
{
	//HAL_UART_Transmit(&huart, (uint8_t*)"AT+RST\r\n", strlen("AT+RST\r\n"), 100); 
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+RST\r\n", strlen("AT+RST\r\n"), 100); 
	HAL_Delay(_ms_); 
}	

void Esp32_Echo(UART_HandleTypeDef huart, FunctionalState ctrl)
{
	switch(ctrl)
	{
		case ENABLE:
			HAL_UART_Transmit(&huart, (uint8_t*)"ATE1\r\n", strlen("ATE1\r\n"), 100); 
			HAL_Delay(_ms_); 
			break; 
		
		case DISABLE:
			HAL_UART_Transmit(&huart, (uint8_t*)"ATE0\r\n", strlen("ATE0\r\n"), 100); 
			HAL_Delay(_ms_); 
			break; 
	}
}

void Esp32_WiFiMode(UART_HandleTypeDef huart, Mode_HandleTypeDef mode)
{
	switch(mode)
	{
		case NULL_Mode:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWMODE=0\r\n", strlen("AT+CWMODE=0\r\n"), 100); 
			HAL_Delay(_ms_);
			break; 
		
		case STA_Mode:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWMODE=1\r\n", strlen("AT+CWMODE=1\r\n"), 100); 
			HAL_Delay(_ms_);
			break; 
		
		case AP_Mode:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWMODE=2\r\n", strlen("AT+CWMODE=2\r\n"), 100); 
			HAL_Delay(_ms_);
			break; 
		
		case AP_and_STA_Mode:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWMODE=3\r\n", strlen("AT+CWMODE=3\r\n"), 100); 
			HAL_Delay(_ms_);
			break; 
	}
}

void Esp32_Start_SmartConfig(UART_HandleTypeDef huart, App_HandleTypeDef app)
{
	switch(app)
	{
		case ESP_TOUCH_App:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWSTARTSMART=1\r\n", strlen("AT+CWSTARTSMART=1\r\n"), 100); 
			HAL_Delay(_ms_);
			break; 
		
		case AIRKISS_App:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWSTARTSMART=2\r\n", strlen("AT+CWSTARTSMART=2\r\n"), 100); 
			HAL_Delay(_ms_);
			break;
		
		case TOUTCH_and_AIRKISS_App:
			HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWSTARTSMART=3\r\n", strlen("AT+CWSTARTSMART=3\r\n"), 100); 
			HAL_Delay(_ms_);
			break; 
	}
}

void Esp32_Stop_SmartConfig(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWSTOPSMART\r\n", strlen("AT+CWSTOPSMART\r\n"), 100); 
	HAL_Delay(_ms_);
}

void Esp32_Check_Info_WiFi(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CWJAP?\r\n", strlen("AT+CWJAP?\r\n"), 100); 
	HAL_Delay(_ms_); 
}

void Esp32_Setup_Default(UART_HandleTypeDef huart)
{
  	HAL_UART_Transmit(&huart, (uint8_t*)"AT+RESTORE\r\n", strlen("AT+RESTORE\r\n"), 100); 
  	HAL_Delay(_ms_); 
}

void Esp32_Mqtt_UserConfig(UART_HandleTypeDef huart, uint8_t *id, uint8_t *username, uint8_t *password) 
{
	uint8_t messages[300]; 
	sprintf((char*)messages, "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", id, username, password); 
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
	HAL_Delay(_ms_);
}

void Esp32_Mqtt_Connect_Broker(UART_HandleTypeDef huart, uint8_t *host, uint16_t port)
{
    uint8_t messages[300];
	sprintf((char*)messages, "AT+MQTTCONN=0,\"%s\",%d,0\r\n", host, port);
    HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100);
    HAL_Delay(_ms_);
}

void Esp32_Mqtt_Disconnect_Broker(UART_HandleTypeDef huart)
{
    HAL_UART_Transmit(&huart, (uint8_t*)"AT+MQTTCLEAN=0\r\n", strlen("AT+MQTTCLEAN=0\r\n"), 100); 
    HAL_Delay(_ms_);
}

void Esp32_Mqtt_Subscribe(UART_HandleTypeDef huart, uint8_t *topic)
{
    uint8_t messages[200];
	sprintf((char*)messages, "AT+MQTTSUB=0,\"%s\",1\r\n", topic); 
    HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
	HAL_Delay(_ms_);
}

void Esp32_Mqtt_Publish(UART_HandleTypeDef huart, uint8_t *recv_buf, uint8_t *topic, uint8_t *msg)
{
    uint16_t timeout = 2000; 

    uint8_t messages[100];
	sprintf((char*)messages, "AT+MQTTPUBRAW=0,\"%s\",%d,1,0\r\n", topic, strlen((char*)msg)); 
    HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
    HAL_Delay(200); 
  
    while(1)
    {
        if(strstr((char*)recv_buf, ">")) 
        {
        	HAL_UART_Transmit(&huart, (uint8_t*)msg, strlen((char*)msg), 100); 
        	break; 
        }
        
        HAL_Delay(1); 
        timeout--; 
        if(timeout == 0) HAL_NVIC_SystemReset();
    }
    
    HAL_Delay(_ms_); 
}

