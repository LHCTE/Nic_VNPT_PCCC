/**
  ******************************************************************************
  * @file           : simcomAT.h
  * @brief          : Header for simcomAT.c file  
  *                   This file contains the common defines of the application.
  * @author         : Nichietsu System Development
  ******************************************************************************
  * @note           : Use AT commands 
  ******************************************************************************
  */
#ifndef _SIMCOMAT_H_
#define _SIMCOMAT_H_

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef SIM7020E
#define __SIM7020E__
#else 
#define __SIMA7670C__
#endif

/* Private typedef -----------------------------------------------------------*/
typedef enum { ECHO_DISABLE = 0, ECHO_ENABLE = !ECHO_DISABLE } StateTypeDef;  
typedef enum { SIM7020E_ERROR = 0, SIM7020E_OK = !SIM7020E_ERROR } SIM7020E_StatusTypeDef; 
typedef enum { SIMA7670C_ERROR = 0, SIMA7670C_OK = !SIMA7670C_ERROR } SIMA7670C_StatusTypeDef; 
typedef enum { PDP_CONTEXT_DEACTIVATE = 0, PDP_CONTEXT_ACTIVATE = !PDP_CONTEXT_DEACTIVATE} PDPConextFunc_TypeDef; 

/* Private defines -----------------------------------------------------------*/
#define _ms_ 100 

#define IP_PDPTYPE      ("IP")
#define IPV6_PDPTYPE    ("IPV6")
#define IPV4V6_PDPTYPE  ("IPV4V6") 

/* Private function prototypes -----------------------------------------------*/
#ifdef __SIM7020E__
SIM7020E_StatusTypeDef Sim7020E_Response(uint8_t *recv_buf, uint8_t *content, uint16_t timeout); 
void Sim7020E_Echo(UART_HandleTypeDef huart, uint8_t ctrl); 
void Sim7020E_Mqtt_UserConfig(UART_HandleTypeDef huart, uint8_t *id, uint8_t *username, uint8_t *password);
void Sim7020E_Mqtt_Connect_Broker(UART_HandleTypeDef huart, uint8_t *host, uint16_t port);
void Sim7020E_Mqtt_Disconnect_Broker(UART_HandleTypeDef huart);
void Sim7020E_Mqtt_Subscribe(UART_HandleTypeDef huart, uint8_t *topic);
void Sim7020E_Mqtt_Publish(UART_HandleTypeDef huart, uint8_t *topic, uint8_t *msg);
#else 
SIMA7670C_StatusTypeDef SimA7670C_Response(uint8_t *recv_buf, uint8_t *content, uint16_t timeout); 
int8_t RSSI_Sim4GLTE(UART_HandleTypeDef huart, uint8_t *recv_buf); 
void Is_Insert_Sim_Card(UART_HandleTypeDef huart); 
void SimA7670C_Reset(UART_HandleTypeDef huart); 
void SimA7670C_Echo(UART_HandleTypeDef huart, StateTypeDef ctrl); 
void SimA7670C_PDP_Context_Config(UART_HandleTypeDef huart, uint8_t identifier, uint8_t *pdp_type);
void SimA7670C_PDP_Context_Set(UART_HandleTypeDef huart, uint8_t identifier, PDPConextFunc_TypeDef status); 

void SimA7670C_MQTT_StartService(UART_HandleTypeDef huart); 
void SimA7670C_MQTT_StopService(UART_HandleTypeDef huart); 
void SimA7670C_MQTT_Client_Config(UART_HandleTypeDef huart, uint8_t *client_id); 
void SimA7670C_MQTT_BrokerConnect(UART_HandleTypeDef huart, uint8_t *host, uint16_t port, uint8_t *username, uint8_t *password);
void SimA7670C_MQTT_BrokerDisconnect(UART_HandleTypeDef huart);
void SimA7670C_Mqtt_Subscribe(UART_HandleTypeDef huart, uint8_t *recv_buf, uint8_t *topic);
void SimA7670C_Mqtt_Publish(UART_HandleTypeDef huart, uint8_t *recv_buf, uint8_t *topic, uint8_t *msg);
#endif

#endif /* _SIMCOMAT_H_ */ 

/************************ (C) COPYRIGHT Nichietsu System Development *****END OF FILE****/
