#include "simcomAT.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __SIM7020E__
SIM7020E_StatusTypeDef Sim7020E_Response(uint8_t *recv_buf, uint8_t *content, uint16_t timeout)
{
	while(1)
	{
		uint8_t *_pStr = (uint8_t*)strstr((char*)recv_buf, (char*)content); 
		if(_pStr != 0) return SIM7020E_OK;

		HAL_Delay(1); 
		timeout--;
		if(timeout == 0) return SIM7020E_ERROR; 
	}
}

void Sim7020E_Echo(UART_HandleTypeDef huart, uint8_t ctrl)
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

// "0" mqtt id (0-4), "3" MQTT 3.1, "clientId", "300" keepalive 60*5 = 600s, "1" cleanseason, "0" will flag, "username", "pass"
void Sim7020E_Mqtt_UserConfig(UART_HandleTypeDef huart, uint8_t *id, uint8_t *username, uint8_t *password) 
{
	uint8_t messages[300];
	sprintf((char*)messages, "AT+CMQCON=0,3,\"%s\",300,1,0,\"%s\",\"%s\"\r\n", id, username, password);
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 

  	HAL_Delay(_ms_); 
}

// "host", "port", "6000" time out, "1000", bufsize
void Sim7020E_Mqtt_Connect_Broker(UART_HandleTypeDef huart, uint8_t *host, uint16_t port)
{
	uint8_t messages[300];
	sprintf((char*)messages, "AT+CMQNEW=\"%s\",\"%d\",6000,1000\r\n", host, port);
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100);

  	HAL_Delay(_ms_); 
}

//"0" is mqtt id normal working only 1 MQTT open at same time
void Sim7020E_Mqtt_Disconnect_Broker(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQDISCON=0\r\n", strlen("AT+CMQDISCON=0\r\n"), 100);
	HAL_Delay(_ms_); 
}

//"0" is mqtt id normal working only 1 MQTT open at same time, "1": QoS
void Sim7020E_Mqtt_Subscribe(UART_HandleTypeDef huart, uint8_t *topic)
{
	uint8_t messages[200];
	sprintf((char*)messages, "AT+CMQSUB=0,\"%s\",1\r\n", topic);
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
  	HAL_Delay(_ms_); 
}

//"0" is mqtt id normal working only 1 MQTT open at same time, "1": QoS
void Sim7020E_Mqtt_Publish(UART_HandleTypeDef huart, uint8_t *topic, uint8_t *msg)
{
	uint8_t data2send[500];
	uint8_t messages[1000];

	for (uint16_t i = 0; i < strlen((char*)msg); i++)
	{
		char temp[2]; //2 hex
		sprintf(temp, "%02X", msg[i]);
		strcat((char*)data2send, temp );
	}
	sprintf((char*)messages, "AT+CMQPUB=0,\"%s\",1,0,0,%d,\"%s\"\r\n",topic, strlen((char*)data2send), data2send);
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 

	//for (uint16_t i = 0; i < 500; i++) data2send[i] = '\0';
}

#else 

SIMA7670C_StatusTypeDef SimA7670C_Response(uint8_t *recv_buf, uint8_t *content, uint16_t timeout)
{
	while(1)
	{
		uint8_t *_pStr = (uint8_t*)strstr((char*)recv_buf, (char*)content); 
		if(_pStr != 0) return SIMA7670C_OK;

		HAL_Delay(1); 
		timeout--;
		if(timeout == 0) return SIMA7670C_ERROR; 
	}
}

int8_t RSSI_Sim4GLTE(UART_HandleTypeDef huart, uint8_t *recv_buf)
{
	uint8_t *pStr_Analysis = 0; 
	uint8_t *temp = 0; 
	int8_t val = 0; 

	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CSQ\r\n", strlen("AT+CSQ\r\n"), 100); 
	HAL_Delay(1000); 

	// +1 Remove string ": "
	pStr_Analysis = (uint8_t*)strstr((char*)recv_buf, ":") + 1; 
	for(uint8_t i = 0; i < strlen((char*)pStr_Analysis); i++)
	{
		if(pStr_Analysis[i] == ',') break; 
		else temp[i] = pStr_Analysis[i]; 
	}

	// Convert 'str' type to 'int' type 
	val = (int8_t)atoi((char*)temp);
	return val; 
}

void Is_Insert_Sim_Card(UART_HandleTypeDef huart) 
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CPIN?\r\n", strlen("AT+CPIN?\r\n"), 100); 
	HAL_Delay(_ms_); 
}

void SimA7670C_Reset(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CRESET\r\n", strlen("AT+CRESET\r\n"), 100); 
	HAL_Delay(1000); 
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CRESET\r\n", strlen("AT+CRESET\r\n"), 100); 

	HAL_Delay(_ms_); 
}

void SimA7670C_Echo(UART_HandleTypeDef huart, StateTypeDef ctrl)
{
	switch(ctrl)
	{
		case ECHO_ENABLE:
			HAL_UART_Transmit(&huart, (uint8_t*)"ATE1\r\n", strlen("ATE1\r\n"), 100); 
			break; 
		
		case ECHO_DISABLE:
			HAL_UART_Transmit(&huart, (uint8_t*)"ATE0\r\n", strlen("ATE0\r\n"), 100); 
			break; 
	}

	HAL_Delay(_ms_); 
}

void SimA7670C_PDP_Context_Config(UART_HandleTypeDef huart, uint8_t identifier, uint8_t *pdp_type)
{
	uint8_t _cmd[40]; 
	sprintf((char*)_cmd, "AT+CGDCONT=%d,\"%s\",\"APN\"\r\n", identifier, pdp_type); 
	HAL_UART_Transmit(&huart, _cmd, strlen((char*)_cmd), 100); 

	HAL_Delay(_ms_); 
}

void SimA7670C_PDP_Context_Set(UART_HandleTypeDef huart, uint8_t identifier, PDPConextFunc_TypeDef status)
{
	uint8_t _cmd[20]; 
	sprintf((char*)_cmd, "AT+CGACT=%d,%d\r\n", status, identifier); 
	HAL_UART_Transmit(&huart, _cmd, strlen((char*)_cmd), 100); 

	HAL_Delay(_ms_); 
}

void SimA7670C_MQTT_StartService(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQTTSTART\r\n", strlen("AT+CMQTTSTART\r\n"), 100); 
	HAL_Delay(_ms_); 
}

void SimA7670C_MQTT_StopService(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQTTSTOP\r\n", strlen("AT+CMQTTSTOP\r\n"), 100); 
	HAL_Delay(_ms_); 
}

void SimA7670C_MQTT_Client_Config(UART_HandleTypeDef huart, uint8_t *client_id)
{
	uint8_t _cmd[50]; 
	sprintf((char*)_cmd, "AT+CMQTTACCQ=0,\"%s\",0\r\n", client_id); 
	HAL_UART_Transmit(&huart, _cmd, strlen((char*)_cmd), 100); 

	HAL_Delay(_ms_); 
}

void SimA7670C_MQTT_BrokerConnect(UART_HandleTypeDef huart, uint8_t *host, uint16_t port, uint8_t *username, uint8_t *password)
{
	uint8_t messages[300]; 
	sprintf((char*)messages, "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",60,0,\"%s\",\"%s\"\r\n", host, port, username, password); 
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 

	HAL_Delay(_ms_); 
}

void SimA7670C_MQTT_BrokerDisconnect(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQTTDISC=0,120\r\n", strlen("AT+CMQTTDISC=0,120\r\n"), 100); 
	HAL_Delay(_ms_); 

	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQTTREL=0\r\n", strlen("AT+CMQTTREL=0\r\n"), 100); 
	HAL_Delay(_ms_); 
}

void SimA7670C_Mqtt_Subscribe(UART_HandleTypeDef huart, uint8_t *recv_buf, uint8_t *topic)
{
	uint8_t messages[200];
	uint16_t timeout = 2000; 

	sprintf((char*)messages, "AT+CMQTTSUBTOPIC=0,%d,1\r\n", strlen((char*)topic)); 
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
	HAL_Delay(2); 
	while(1)
    {
        if(strstr((char*)recv_buf, ">")) 
        {
        	HAL_UART_Transmit(&huart, (uint8_t*)topic, strlen((char*)topic), 100); 
        	break; 
        }
        
        HAL_Delay(1); 
        timeout--; 
        if(timeout == 0) HAL_NVIC_SystemReset();
    }

	// Remove and use continue
	if(!SimA7670C_Response(recv_buf, (uint8_t*)"OK", 10000)) HAL_NVIC_SystemReset(); 
    else for(uint16_t pos = 0; pos < 200; pos++) messages[pos] = '\0'; 	
	HAL_Delay(_ms_); 

	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQTTSUB=0\r\n", strlen("AT+CMQTTSUB=0\r\n"), 100); 
	HAL_Delay(2); 

	// Remove and use continue
	if(!SimA7670C_Response(recv_buf, (uint8_t*)"+CMQTTSUB:", 10000)) HAL_NVIC_SystemReset(); 
    else for(uint16_t pos = 0; pos < 200; pos++) messages[pos] = '\0'; 
	HAL_Delay(_ms_); 
}

void SimA7670C_Mqtt_Publish(UART_HandleTypeDef huart, uint8_t *recv_buf, uint8_t *topic, uint8_t *msg)
{
	uint8_t messages[800];
	uint16_t timeout = 2000; 

	/*sprintf((char*)messages, "AT+CMQTTSUB=0,%d,1\r\n", strlen((char*)topic)); 
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
	HAL_Delay(2); 
	while(1)
    {
        if(strstr((char*)recv_buf, ">")) 
        {
        	HAL_UART_Transmit(&huart, topic, strlen((char*)topic), 100); 
        	break; 
        }
        
        HAL_Delay(1); 
        timeout--; 
        if(timeout == 0) HAL_NVIC_SystemReset();
    }

	// Remove and use continue
	if(!SimA7670C_Response(recv_buf, (uint8_t*)"+CMQTTSUB", 10000)) HAL_NVIC_SystemReset(); 
    else for(uint16_t pos = 0; pos < 800; pos++) messages[pos] = '\0'; 
	timeout = 2000; 
	HAL_Delay(_ms_); */ 

	sprintf((char*)messages, "AT+CMQTTTOPIC=0,%d\r\n", strlen((char*)topic)); 
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
	HAL_Delay(2); 
	while(1)
    {
        if(strstr((char*)recv_buf, ">")) 
        {
        	HAL_UART_Transmit(&huart, topic, strlen((char*)topic), 100); 
        	break; 
        }
        
        HAL_Delay(1); 
        timeout--; 
        if(timeout == 0) HAL_NVIC_SystemReset();
	}

	// Remove and use continue
	if(!SimA7670C_Response(recv_buf, (uint8_t*)"OK", 10000)) HAL_NVIC_SystemReset(); 
    else for(uint16_t pos = 0; pos < 800; pos++) messages[pos] = '\0'; 
	timeout = 2000;
	HAL_Delay(_ms_); 

	sprintf((char*)messages, "AT+CMQTTPAYLOAD=0,%d\r\n", strlen((char*)msg)); 
	HAL_UART_Transmit(&huart, messages, strlen((char*)messages), 100); 
	HAL_Delay(2); 
	while(1)
    {
        if(strstr((char*)recv_buf, ">")) 
        {
        	HAL_UART_Transmit(&huart, msg, strlen((char*)msg), 100); 
        	break; 
        }
        
        HAL_Delay(1); 
        timeout--; 
        if(timeout == 0) HAL_NVIC_SystemReset();
	}
	if(!SimA7670C_Response(recv_buf, (uint8_t*)"OK", 10000)) HAL_NVIC_SystemReset(); 
    else for(uint16_t pos = 0; pos < 800; pos++) messages[pos] = '\0'; 
	HAL_Delay(_ms_); 

	HAL_UART_Transmit(&huart, (uint8_t*)"AT+CMQTTPUB=0,1,60\r\n", strlen("AT+CMQTTPUB=0,1,60\r\n"), 100); 
	HAL_Delay(2); 

	// Remove and use continue
	if(!SimA7670C_Response(recv_buf, (uint8_t*)"+CMQTTPUB:", 10000)) HAL_NVIC_SystemReset(); 
    else for(uint16_t pos = 0; pos < 800; pos++) messages[pos] = '\0'; 
	HAL_Delay(_ms_); 
} 

#endif
