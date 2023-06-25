/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h> 
#include "datatype.h"
#include "eeprom.h"
#include "modbusRTU.h"
#include "relay.h"

//...if you use SIMA7670C peripheral then commnent this command 
#define SIMA7670C 
#include "simcomAT.h" 

//...if you use ESP8266 peripheral then commnent this command 
#define ESP32 
#include "espressifAT.h" 
#define CBSC
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { WIFI_DISCONNECTED = 0, WIFI_CONNECTED = !WIFI_DISCONNECTED } WiFi_StatusTypeDef; 
typedef enum { SIM_NOT_INSERTED = 0, SIM_INSERTING = !SIM_NOT_INSERTED} SimCard_StatusTypeDef; 
typedef enum { INVALID_SYNTAX = 0, CORRECT_SYNTAX = !INVALID_SYNTAX } Syntax_HandleTypeDef; 

typedef enum 
{
    //...None Configure
    NONE_CONFIG                 = 0,
    SERVERCMD_RESTART           = 1,
    SERVERCMD_BREAK             = 2, 

    //...Configure device
    SERVERCMD_WRITECFG_RS485    = 3, 
    SERVERCMD_WRITECFG_INTERVAL = 4,
    SERVERCMD_WRITECFG_BAUDRATE = 5,
    SERVERCMD_WRITECFG_TIMEC2C  = 6, 

    //...Configure Broker  
    SERVERCMD_WRITECFG_HOST     = 7, 
    SERVERCMD_WRITECFG_PORT     = 8, 
    SERVERCMD_WRITECFG_ID       = 9, 
    SERVERCMD_WRITECFG_USERNAME = 10, 
    SERVERCMD_WRITECFG_PASSWORD = 11, 
    SERVERCMD_WRITECFG_TOPICSUB = 12, 
    SERVERCMD_WRITECFG_TOPICPUB = 13, 

    //...Goto Configure Mode
    SERVERCMD_GOTOCFG_RS485     = 14,
    SERVERCMD_GOTOCFG_INTERVAL  = 15,
    SERVERCMD_GOTOCFG_BAUDRATE  = 16,  
    SERVERCMD_GOTOCFG_TIMEC2C   = 17,
    SERVERCMD_GOTOCFG_BROKER    = 18 
} ServerCmd_ConfigModHandleTypeDef;

typedef enum 
{
  //...None Command 
  NONE_CMD                   	= 0,

  //...Command
  SERVERCMD_NORMAL_RS485     	= 1, 
  SERVERCMD_NORMAL_PING      	= 2,
  SERVERCMD_NORMAL_QUEST_CMD 	= 3,
  SERVERCMD_NORMAL_RESTART   	= 4,
  SERVERCMD_NORMAL_RELAY_0	  = 5,
	SERVERCMD_NORMAL_RELAY_1	  = 6,
	SERVERCMD_NORMAL_CONFIRM    = 7,
  SERVERCMD_NORMAL_SIM_RSSI   = 8,
  SERVERCMD_NORMAL_WIRELESS   = 9 
} ServerCmd_NormalModHandleTypeDef; 

typedef enum 
{
  SYSMONITOR_CONFIGRS485_MODE     = 1,
  SYSMONITOR_CONFIGCMD2CMD_MODE   = 2, 
  SYSMONITOR_CONFIGINTERVAL_MODE  = 3, 
  SYSMONITOR_CONFIGBAUDRATE_MODE  = 4,
  SYSMONITOR_CONFIGBROKER_MODE    = 5, 
  SYSMONITOR_NORMAL_MODE          = 6
} SysMonitor_ModeHandleTypeDef; 

typedef enum
{
  None_Modules 		  = 0, 
  Esp32_Modules 	  = 1,
  SimA7670C_Modules  = 2 
} Module_PeripheralsTypeDef;
typedef enum
{
  PRODUCT_SERVER      = 0,
	DEVELOPMENT_SERVER  = 1,
  HYTEK_SERVER	      = 2 
} Server_OptionTypeDef; 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
MQTT_StructuresTypeDef broker;   
Module_PeripheralsTypeDef wirelessProtocol;

//...Buffer data of IoT_Protocol
uint8_t IoT_Protocol_Buf[IoT_Byte]; 
uint8_t IoT_Protocol_Data = 0; 
uint16_t IoT_Protocol_Count = 0;

//...Buffer data of Modbus Protocol
uint8_t Inverter_Buf[Inverter_Byte]; 
uint8_t Inverter_Data = 0; 
uint16_t Inverter_Count = 0; 

//...Buffer data of Eeprom
uint32_t Eeprom_Buf[Eeprom_Byte]; 

uint32_t Eeprom_Host[Eeprom_Host_Byte];
uint32_t Eeprom_Username[Eeprom_Username_Byte];
uint32_t Eeprom_Password[Eeprom_Password_Byte];   
uint32_t Eeprom_TopicSub[Eeprom_TopicSub_Byte]; 
uint32_t Eeprom_TopicPub[Eeprom_TopicPub_Byte]; 

uint8_t Send_Package_Buf[Send_Byte];
uint8_t Receive_Package_Buf[Receive_Byte];
uint8_t Filter_Buf[Filter_Byte]; 

uint8_t WIRELESS_DISPLAY = 0, DATA_DISPLAY = 0;
uint8_t Interval_Flag = 0, Req_Ping_Flag = 0; 
uint32_t Baudrate_Default = 9600; 

uint32_t intervalValue = 0; // bien nay dung de gui data sau 1 khoan thoi gian
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
ServerCmd_ConfigModHandleTypeDef Configuration_Commands_Handle(uint8_t *recv_buf, uint16_t size); 
ServerCmd_NormalModHandleTypeDef Normal_CommandsHandle(uint8_t *recv_buf, uint16_t size); 
Syntax_HandleTypeDef Is_Setup_Right(uint8_t *recv_buf, uint16_t size); 
SysMonitor_ModeHandleTypeDef Waiting_For_Config(uint8_t *recv_buf, uint16_t size, uint16_t timeout); 
WiFi_StatusTypeDef Stm32_Check_Esp32_Connect_WiFi(uint16_t timeout); 
SimCard_StatusTypeDef Stm32_Check_Sim_Card_Connect_Network_Service(uint16_t timeout); 

void Esp32_WiFi_Init(void); 
void SimA7670C_4GLTE_Init(void); 
void Stm32_Restart(void); 
void Stm32_Clear_Buf(uint8_t *clr_buf, uint16_t size); 
void Checking_Wireless_Response(uint8_t *recv_buf, uint8_t *content);
void Broker_Connect_Init(Server_OptionTypeDef server_name, Module_PeripheralsTypeDef modules); 
void Broker_Publish_Topic(uint8_t *msg); 
void Broker_Subscribe_Topic(void); 
unsigned char CheckInput(void);
void sendCBSC (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{ 
	UNUSED(huart); 

	//...Inverter
	if(huart->Instance == USART1) 
  {
    HAL_TIM_Base_Start(&htim15);
    HAL_UART_Receive_IT(&huart1, &Inverter_Data, 1);
    Inverter_Buf[Inverter_Count++] = Inverter_Data;
		if(Inverter_Count > Inverter_Byte - 1) Inverter_Count = 0;  

    __HAL_TIM_SET_COUNTER(&htim15, 0);    
	}

	//...Sim 4G-LTE
	if(huart->Instance == USART2) 
	{
    HAL_TIM_Base_Start(&htim6);
		HAL_UART_Receive_IT(&huart2, &IoT_Protocol_Data, 1);
		if(IoT_Protocol_Data == 0) IoT_Protocol_Data = 64; 
		IoT_Protocol_Buf[IoT_Protocol_Count++] = IoT_Protocol_Data; 
		if(IoT_Protocol_Count > IoT_Byte - 1) IoT_Protocol_Count = 0; 

    __HAL_TIM_SET_COUNTER(&htim6, 0);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	UNUSED(htim);

  //...Timer send data from memory
  static uint8_t Interval_Count = 0; 
	static uint8_t Button_Press_Keep_Count = 0; 
	static uint8_t Req_Ping_Count = 0;  

  if(htim->Instance == TIM3) 
	{
    //...Interval Timer 
    Interval_Count++; 
    if(Interval_Count > intervalValue)
    {
      Interval_Flag = 1; 
      Interval_Count = 0; 
    }
		
    //...Check reset wifi 
		if(HAL_GPIO_ReadPin(WiFi_Reset_GPIO_Port, WiFi_Reset_Pin) == GPIO_PIN_RESET)
		{
			Button_Press_Keep_Count++; 
			if(Button_Press_Keep_Count == 3)
			{
				Esp32_Setup_Default(huart3); 
				HAL_NVIC_SystemReset();
			}
		}
		
    //...Request ping keep alive of device 
		Req_Ping_Count++; 
		if(Req_Ping_Count == 240) 
		{
			Req_Ping_Flag = 1; 
			Req_Ping_Count = 0; 
		}
  }

	//...Timer light display 
	static uint8_t State_WiFi = 0; 
	static uint8_t State_Data = 0;
	static uint8_t Previous_State = 0;
	
	if(htim->Instance == TIM1) 
  {
		//...LED WIFI
		State_WiFi++; 
		if(State_WiFi == WIRELESS_DISPLAY) 
		{
			if(WIRELESS_DISPLAY == Connected_WiFi) HAL_GPIO_WritePin(Battery_LED_GPIO_Port, Battery_LED_Pin, GPIO_PIN_RESET);
			else HAL_GPIO_TogglePin(Battery_LED_GPIO_Port, Battery_LED_Pin);
			State_WiFi = 0; 
		}
		
		//...LED DATA
		if(Previous_State != DATA_DISPLAY) 
		{
			Previous_State = DATA_DISPLAY;
			State_Data = 0;
		}
    
		State_Data++; 
		if(State_Data == DATA_DISPLAY) 
		{
			if(DATA_DISPLAY == Check_Mode) HAL_GPIO_WritePin(Test_LED_GPIO_Port, Test_LED_Pin, GPIO_PIN_SET); 
			else if(DATA_DISPLAY == Config_Mode) HAL_GPIO_WritePin(Test_LED_GPIO_Port, Test_LED_Pin, GPIO_PIN_RESET); 
			else HAL_GPIO_TogglePin(Test_LED_GPIO_Port, Test_LED_Pin);
			State_Data = 0; 
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if(Eeprom_Read_Data(Addr_Baudrate, Position_1) != 0xFFFFFFFF) Baudrate_Default = Eeprom_Read_Data(Addr_Baudrate, Position_1); 
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &Inverter_Data, 1); 
	HAL_UART_Receive_IT(&huart2, &IoT_Protocol_Data, 1);
  HAL_TIM_Base_Start_IT(&htim1); 
  HAL_TIM_Base_Start_IT(&htim3); 
	HAL_Delay(100);	

  // if(HAL_GPIO_ReadPin(Checking_Esp32_GPIO_Port, Checking_Esp32_Pin) == GPIO_PIN_SET)
	// {
  //   wirelessProtocol = Esp32_Modules; 
  //   Esp32_WiFi_Init();
  // }
  // else if(HAL_GPIO_ReadPin(Checking_Esp32_GPIO_Port, Checking_Esp32_Pin) == GPIO_PIN_RESET)
  // {
  //   wirelessProtocol = SimA7670C_Modules; 
  //   SimA7670C_4GLTE_Init(); 
  // }
  // else
  // {
  //   while(1)
  //   {
  //     HAL_GPIO_WritePin(Test_LED_GPIO_Port, Test_LED_Pin, GPIO_PIN_RESET); 
  //     HAL_GPIO_WritePin(Battery_LED_GPIO_Port, Battery_LED_Pin, GPIO_PIN_RESET); 
  //     HAL_GPIO_WritePin(DC_LED_GPIO_Port, DC_LED_Pin, GPIO_PIN_RESET); 
  //     HAL_Delay(1000); 
  //     HAL_GPIO_WritePin(Test_LED_GPIO_Port, Test_LED_Pin, GPIO_PIN_SET); 
  //     HAL_GPIO_WritePin(Battery_LED_GPIO_Port, Battery_LED_Pin, GPIO_PIN_SET); 
  //     HAL_GPIO_WritePin(DC_LED_GPIO_Port, DC_LED_Pin, GPIO_PIN_SET);
  //     HAL_Delay(1000); 
  //   }
  // }

  // while(1)
  // {
  //   HAL_UART_Transmit(&huart2, (uint8_t*)"AT+CSQ\r\n", strlen("AT+CSQ\r\n"), 100); 
	//   HAL_Delay(2000); 
  // }
  
  wirelessProtocol = SimA7670C_Modules; 
  SimA7670C_4GLTE_Init(); 
  
  Broker_Connect_Init(Server,  wirelessProtocol); 
  HAL_GPIO_WritePin(DC_LED_GPIO_Port, DC_LED_Pin, GPIO_PIN_RESET); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t Set_Time = 30;
    uint8_t Timeout_UserConfig = Set_Time;  
    uint8_t *Analysis = 0, Mode_Flag = 0, CmdConfig_Cnt = 0;
    uint16_t CRC_Byte = 0, Config_Addr_Next = 0; 
    uint32_t BaudRate_Value = 0, Interval_Value = 0, SetTime_C2C = 0; 

    Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
    Mode_Flag = Waiting_For_Config(IoT_Protocol_Buf, IoT_Byte, 8000); 

    if(Mode_Flag == SYSMONITOR_CONFIGBROKER_MODE)
    {
      //...Clear page 
      Eeprom_ClearPage(User_ConfigInfomation_Page); 
  
      sprintf((char*)Filter_Buf, "%s", "<!> Started config broker"); 
      Broker_Publish_Topic(Filter_Buf); 
			Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
			Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
			HAL_Delay(100);

      Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
      Timeout_UserConfig = Set_Time;
      while(1)
      {
        DATA_DISPLAY = Config_Mode; 
        uint8_t cmd = Configuration_Commands_Handle(IoT_Protocol_Buf, IoT_Byte); 
        
        //...Wait for the end of string
				while(cmd != NONE_CMD)
        {
          if(__HAL_TIM_GetCounter(&htim6) > 5)
          {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Stop(&htim6);

            break; 
          }
        } 

        switch(cmd)
        {
          case SERVERCMD_WRITECFG_HOST:
            //...+5 Remove string "HOST:"
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "HOST") + 5;  

            //...Filter string 
						for(uint8_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0' || Analysis[i] == 0) break; 
							else Filter_Buf[i] = Analysis[i];						
						}

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte);
            
            TextToHexStr(Filter_Buf, Receive_Package_Buf); 
            HexStrToHexChar(Receive_Package_Buf, Send_Package_Buf);
            if(Eeprom_WriteStr_Hexadecimal(Addr_Host, (uint64_t*)Send_Package_Buf, 1000) == FLASH_OK)
            {
              Broker_Publish_Topic((uint8_t*)"<!> Broker-Host Write Ok");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Broker-Host Write Error");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }

            Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Send_Byte);
            Timeout_UserConfig = Set_Time; 
            break; 

          case SERVERCMD_WRITECFG_PORT: 
            if(Is_Setup_Right(IoT_Protocol_Buf, IoT_Byte) == CORRECT_SYNTAX)
            {
              //...+5 Remove string "PORT:"
              Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "PORT") + 5;

              if(Eeprom_Write_Data(Addr_Port, Position_2, (uint32_t)atoi((char*)Analysis), 1000) == FLASH_OK)
              {
                Broker_Publish_Topic((uint8_t*)"<!> Broker-Port Write Ok"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Broker-Port Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Invalid Broker-Port Syntax");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            } 

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            Timeout_UserConfig = Set_Time;
            break;

          case SERVERCMD_WRITECFG_ID: 
            if(Is_Setup_Right(IoT_Protocol_Buf, IoT_Byte) == CORRECT_SYNTAX)
            {
              //...+3 Remove string "ID:"
              Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "ID") + 3;

              if(Eeprom_Write_Data(Addr_ID, Position_1, (uint32_t)atoi((char*)Analysis), 1000) == FLASH_OK)
              {
                Broker_Publish_Topic((uint8_t*)"<!> Broker-ID Write Ok");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Broker-ID Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Invalid Broker-ID Syntax");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            } 
            
            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            Timeout_UserConfig = Set_Time;
            break;

          case SERVERCMD_WRITECFG_USERNAME:
            //...+9 Remove string "USERNAME:"
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "USERNAME") + 9;

            //...Filter string 
						for(uint8_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0' || Analysis[i] == 0) break; 
							else Filter_Buf[i] = Analysis[i];						
						}

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 

            TextToHexStr(Filter_Buf, Receive_Package_Buf); 
            HexStrToHexChar(Receive_Package_Buf, Send_Package_Buf);
            if(Eeprom_WriteStr_Hexadecimal(Addr_Username, (uint64_t*)Send_Package_Buf, 1000) == FLASH_OK)
            {
              Broker_Publish_Topic((uint8_t*)"<!> Broker-Username Write Ok");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Broker-Username Write Error");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }

            Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Send_Byte); 
            Timeout_UserConfig = Set_Time;
            break;

          case SERVERCMD_WRITECFG_PASSWORD:
            //...+9 Remove string "PASSWORD:"
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "PASSWORD") + 9;

            //...Filter string 
						for(uint8_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0' || Analysis[i] == 0) break; 
							else Filter_Buf[i] = Analysis[i];						
						}

						Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 

            //...Write Password to Eeprom 
            TextToHexStr(Filter_Buf, Receive_Package_Buf); 
            HexStrToHexChar(Receive_Package_Buf, Send_Package_Buf);
            if(Eeprom_WriteStr_Hexadecimal(Addr_Password, (uint64_t*)Send_Package_Buf, 1000) == FLASH_OK)
            {
              Broker_Publish_Topic((uint8_t*)"<!> Broker-Password Write Ok");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            else
            {
              Broker_Publish_Topic((uint8_t*)"<X> Broker-Password Write Error");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }

            Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Send_Byte); 
            Timeout_UserConfig = Set_Time;
            break;

          case SERVERCMD_WRITECFG_TOPICSUB:
            //...+9 Remove string "TOPICSUB:"
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "TOPICSUB") + 9;

            //...Filter string 
						for(uint8_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0' || Analysis[i] == 0) break; 
							else Filter_Buf[i] = Analysis[i];						
						}

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 

            TextToHexStr(Filter_Buf, Receive_Package_Buf); 
            HexStrToHexChar(Receive_Package_Buf, Send_Package_Buf);
            if(Eeprom_WriteStr_Hexadecimal(Addr_TopicSub, (uint64_t*)Send_Package_Buf, 1000) == FLASH_OK)
            {
              Broker_Publish_Topic((uint8_t*)"<!> Broker-Topic Subscribe Write Ok");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            else
            {
              Broker_Publish_Topic((uint8_t*)"<X> Broker-Topic Subscribe Write Error");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            
            Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Send_Byte); 
            Timeout_UserConfig = Set_Time;
            break;
          
          case SERVERCMD_WRITECFG_TOPICPUB:
            //...+9 Remove string "TOPICPUB:"
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "TOPICPUB") + 9;

            //...Filter string 
						for(uint8_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0' || Analysis[i] == 0) break; 
							else Filter_Buf[i] = Analysis[i];						
						}

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 

            TextToHexStr(Filter_Buf, Receive_Package_Buf); 
            HexStrToHexChar(Receive_Package_Buf, Send_Package_Buf);
            if(Eeprom_WriteStr_Hexadecimal(Addr_TopicPub, (uint64_t*)Send_Package_Buf, 1000) == FLASH_OK)
            {
              Broker_Publish_Topic((uint8_t*)"<!> Broker-Topic Publish Write Ok");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            else
            {
              Broker_Publish_Topic((uint8_t*)"<X> Broker-Topic Publish Write Error");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }

            Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            Stm32_Clear_Buf(Receive_Package_Buf, Send_Byte);  
            Timeout_UserConfig = Set_Time;
            break; 

          case SERVERCMD_BREAK:
            if(Eeprom_Read_Data(Addr_Host, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_Port, Position_2) != 0xFFFFFFFF && 
                Eeprom_Read_Data(Addr_ID, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_Username, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_Password, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_TopicPub, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_TopicSub, Position_1) != 0xFFFFFFFF) 
            {
              if(Eeprom_Write_Data(Addr_BrokerFlag, Position_1, CmdConfig_Cnt, 1000) == FLASH_OK)
              {
                Broker_Publish_Topic((uint8_t*)"<!> Stopped configure -> Restart now");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Write Error"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
              }
            }
            else
            {
              Broker_Publish_Topic((uint8_t*)"<!> None Configure -> Restart now");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }

            HAL_NVIC_SystemReset(); 
            break; 
        }

        HAL_Delay(1000); 
        Timeout_UserConfig--; 
        if(Timeout_UserConfig == 0)
        {
          if(Eeprom_Read_Data(Addr_Host, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_Port, Position_2) != 0xFFFFFFFF && 
                Eeprom_Read_Data(Addr_ID, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_Username, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_Password, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_TopicPub, Position_1) != 0xFFFFFFFF && \
                Eeprom_Read_Data(Addr_TopicSub, Position_1) != 0xFFFFFFFF) 
          {
            if(Eeprom_Write_Data(Addr_BrokerFlag, Position_1, CmdConfig_Cnt, 1000) == FLASH_OK)
            {
              Broker_Publish_Topic((uint8_t*)"<!> Configure Timeout -> Restart now");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Write Error"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }
          }
          else
          {
            Broker_Publish_Topic((uint8_t*)"<!> None Configure -> Restart now");
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
          }

          Mode_Flag = SYSMONITOR_NORMAL_MODE; 
          break; 
        }
      }
    }
    else if(Mode_Flag == SYSMONITOR_CONFIGINTERVAL_MODE)
    {
      sprintf((char*)Filter_Buf, "%s", "<!> Started config interval"); 
      Broker_Publish_Topic(Filter_Buf); 
			Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
			Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
			HAL_Delay(100); 

      Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
      Timeout_UserConfig = Set_Time;
      while(1)
      {
        DATA_DISPLAY = Config_Mode; 
        uint8_t cmd = Configuration_Commands_Handle(IoT_Protocol_Buf, IoT_Byte); 
        
        //...Wait for the end of string
				while(cmd != NONE_CMD)
        {
          if(__HAL_TIM_GetCounter(&htim6) > 5)
          {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Stop(&htim6);

            break; 
          }
        } 

        switch(cmd)
        {
          case SERVERCMD_WRITECFG_INTERVAL:
            if(Is_Setup_Right(IoT_Protocol_Buf, IoT_Byte) == CORRECT_SYNTAX)
            {
              //...+9 Remove string "INTERVAL="
              Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "INTERVAL") + 9;  

              if(wirelessProtocol == Esp32_Modules) Interval_Value = atoi((char*)Analysis); 
              else if(wirelessProtocol == SimA7670C_Modules) 
              {
                uint8_t tempo[100]; 
                for(uint8_t i = 0; i < 100; i++)
                {
                  if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '+') break; 
                  else tempo[i] = Analysis[i];						

                  //for(uint8_t j = 0; j < strlen((char*)tempo); j++) if((tempo[j] < 48) || (tempo[j] > 57)) return INVALID_SYNTAX; 
                }

                Interval_Value = atoi((char*)tempo); 
              }

              if(Interval_Value < 10) Interval_Value = 10; //...Limit Interval
              else if(Interval_Value > 300) Interval_Value = 60; //...Too Interval 

              if(Eeprom_Read_Data(Addr_Interval, Position_1) == 0xFFFFFFFF || Eeprom_Read_Data(Addr_Interval, Position_1) != 0xFFFFFFFF) 
              {
                //...Clear page 
                Eeprom_ClearPage(User_ConfigInterval_Page); 

                //...Check write data 
                if(Eeprom_Write_Data(Addr_Interval, Position_1, Interval_Value, 1000) == FLASH_OK) 
                {
                  if(Eeprom_Read_Data(Addr_Interval, Position_1) != 0xFFFFFFFF) 
                  {
                    Broker_Publish_Topic((uint8_t*)"<!> Interval Write Ok -> Restart now");
                    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                    HAL_NVIC_SystemReset(); 
                  }    
                  else 
                  {
                    Broker_Publish_Topic((uint8_t*)"<X> Interval Write Error");
                    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                  }
                }
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Invalid Interval Syntax");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            } 

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            break; 

          //...If commands then clear 
          case SERVERCMD_GOTOCFG_RS485:
          case SERVERCMD_GOTOCFG_INTERVAL:
          case SERVERCMD_GOTOCFG_BAUDRATE:
          case SERVERCMD_GOTOCFG_TIMEC2C:
          case SERVERCMD_WRITECFG_RS485:
          case SERVERCMD_WRITECFG_BAUDRATE:
          case SERVERCMD_WRITECFG_TIMEC2C:
          case SERVERCMD_RESTART:
          case SERVERCMD_BREAK:
            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            break; 
        }

        HAL_Delay(1000); 
        Timeout_UserConfig--; 
        if(Timeout_UserConfig == 0)
        {
          if(Eeprom_Read_Data(Addr_Interval, Position_1) == 0xFFFFFFFF) 
          {
            //...Value Interval Default
            const uint8_t Interval_Default = 45;

            //...Clear page 
            Eeprom_ClearPage(User_ConfigInterval_Page); 

            //...Check write data 
            if(Eeprom_Write_Data(Addr_Interval, Position_1, Interval_Default, 1000) == FLASH_OK) 
            {
              if(Eeprom_Read_Data(Addr_Interval, Position_1) != 0xFFFFFFFF) 
              {
                Broker_Publish_Topic((uint8_t*)"<!> Interval Default Write Ok"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Interval Default Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Interval Already exists"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
          }
          else 
          {
            Broker_Publish_Topic((uint8_t*)"<!> Timeout!"); 
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
          }

          Mode_Flag = SYSMONITOR_NORMAL_MODE; 
          break; 
        }
      }
    } 
    else if(Mode_Flag == SYSMONITOR_CONFIGCMD2CMD_MODE)
    {
      sprintf((char*)Filter_Buf, "%s", "<!> Started config Command to Command"); 
      Broker_Publish_Topic(Filter_Buf); 
			Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
			Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
			HAL_Delay(100); 

      Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
      Timeout_UserConfig = Set_Time; 
      while(1)
      {
        DATA_DISPLAY = Config_Mode; 
        uint8_t cmd = Configuration_Commands_Handle(IoT_Protocol_Buf, IoT_Byte); 

        //...Wait for the end of string
			  while(cmd != NONE_CMD)
        {
          if(__HAL_TIM_GetCounter(&htim6) > 5)
          {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Stop(&htim6);

            break; 
          }
        } 

        switch(cmd)
        {
          case SERVERCMD_WRITECFG_TIMEC2C: 
            if(Is_Setup_Right(IoT_Protocol_Buf, IoT_Byte) == CORRECT_SYNTAX)
            {
              //...+8 Remove string "CMDSEND=" 
              Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "CMDSEND") + 8;  
              SetTime_C2C = atoi((char*)Analysis); 
              
              //...From 200 - 500 milliseconds 
              if(SetTime_C2C < 200) SetTime_C2C = 200; //...Limit Time Cmd 2 Cmd 
              else if(SetTime_C2C > 500) SetTime_C2C = 500; //...Too Time Cmd 2 Cmd 

              if(Eeprom_Read_Data(Addr_C2C, Position_1) == 0xFFFFFFFF || Eeprom_Read_Data(Addr_C2C, Position_1) != 0xFFFFFFFF) 
              {
                //...Clear page 
                Eeprom_ClearPage(User_ConfigCmd2Cmd_Page); 

                //...Check write data 
                if(Eeprom_Write_Data(Addr_C2C, Position_1, SetTime_C2C, 1000) == FLASH_OK)
								{
                  if(Eeprom_Read_Data(Addr_C2C, Position_1) != 0xFFFFFFFF) 
                  {
                    Broker_Publish_Topic((uint8_t*)"<!> Time Cmd 2 Cmd Write Ok -> Restart now"); 
                    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                    HAL_NVIC_SystemReset(); 
                  }
                  else 
                  {
                    Broker_Publish_Topic((uint8_t*)"<X> Time Cmd 2 Cmd Write Error");
                    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                  }
                }
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Invalid Time Cmd 2 Cmd Syntax");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            break; 

          //...If commands then clear  
          case SERVERCMD_GOTOCFG_RS485:
          case SERVERCMD_GOTOCFG_INTERVAL:
          case SERVERCMD_GOTOCFG_BAUDRATE:
          case SERVERCMD_GOTOCFG_TIMEC2C:
          case SERVERCMD_WRITECFG_RS485:
          case SERVERCMD_WRITECFG_INTERVAL:
          case SERVERCMD_WRITECFG_BAUDRATE:
          case SERVERCMD_RESTART:
          case SERVERCMD_BREAK:
						Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						break; 
        }

        HAL_Delay(1000); 
        Timeout_UserConfig--; 
        if(Timeout_UserConfig == 0)
        {
          if(Eeprom_Read_Data(Addr_C2C, Position_1) == 0xFFFFFFFF) 
          {
            //...Variable default 
            const uint8_t TimeCmd2Cmd_Default = 200;  

            //...Clear page 
            Eeprom_ClearPage(User_ConfigCmd2Cmd_Page); 

            //...Check write data 
            if(Eeprom_Write_Data(Addr_C2C, Position_1, TimeCmd2Cmd_Default, 1000) == FLASH_OK)
            {
              if(Eeprom_Read_Data(Addr_C2C, Position_1) != 0xFFFFFFFF) 
              {
                Broker_Publish_Topic((uint8_t*)"<!> Time Cmd 2 Cmd Default Write Ok"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Time Cmd 2 Cmd Default Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Time Cmd 2 Cmd Already exists"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
          }
          else 
          {
            Broker_Publish_Topic((uint8_t*)"<!> Timeout!"); 
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
          }

          Mode_Flag = SYSMONITOR_NORMAL_MODE; 
          break; 
        }
      }
    }
    else if(Mode_Flag == SYSMONITOR_CONFIGBAUDRATE_MODE)
    {
      sprintf((char*)Filter_Buf, "%s", "<!> Started config baud rate"); 
      Broker_Publish_Topic(Filter_Buf); 
			Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
			Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
			HAL_Delay(100);

      Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
      Timeout_UserConfig = Set_Time; 
      while(1)
      {
        DATA_DISPLAY = Config_Mode; 
        uint8_t cmd = Configuration_Commands_Handle(IoT_Protocol_Buf, IoT_Byte); 

        //...Wait for the end of string
				while(cmd != NONE_CMD)
        {
          if(__HAL_TIM_GetCounter(&htim6) > 5)
          {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Stop(&htim6);

            break; 
          }
        } 

        switch(cmd)
        {
          case SERVERCMD_WRITECFG_BAUDRATE:
            if(Is_Setup_Right(IoT_Protocol_Buf, IoT_Byte) == CORRECT_SYNTAX)
            {
              Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "BAUDRATE") + 9; //+9: Remove string "BAUDRATE="  
              BaudRate_Value = atoi((char*)Analysis); 
              
              /*if((BaudRate_Value == 9600) || (BaudRate_Value == 115200))
              {
                if(Eeprom_Read_Data(Addr_Baudrate, Position_1) == 0xFFFFFFFF || Eeprom_Read_Data(Addr_Baudrate, Position_1) != 0xFFFFFFFF) 
                {
                  //...Clear page 
                  Eeprom_ClearPage(User_ConfigBaudrate_Page); 

                  //...Check write data 
                  if(Eeprom_Write_Data(Addr_Baudrate, Position_1, BaudRate_Value, 1000) == FLASH_OK) 
                  {
                    if(Eeprom_Read_Data(Addr_Baudrate, Position_1) != 0xFFFFFFFF)
                    {
                      Broker_Publish_Topic((uint8_t*)"<!> Baud Rate Write Ok -> Restart now"); 
                      Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                      HAL_NVIC_SystemReset(); 
                    }
                    else 
                    {
                      Broker_Publish_Topic((uint8_t*)"<X> Baud Rate Write Error");
                      Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                    }
                  }
                }
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> None. Only baud rate: 9600bps or 115200bps"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }*/

              if(Eeprom_Read_Data(Addr_Baudrate, Position_1) == 0xFFFFFFFF || Eeprom_Read_Data(Addr_Baudrate, Position_1) != 0xFFFFFFFF) 
              {
                //...Clear page 
                Eeprom_ClearPage(User_ConfigBaudrate_Page); 

                //...Check write data 
                if(Eeprom_Write_Data(Addr_Baudrate, Position_1, BaudRate_Value, 1000) == FLASH_OK) 
                {
                  if(Eeprom_Read_Data(Addr_Baudrate, Position_1) != 0xFFFFFFFF)
                  {
                    Broker_Publish_Topic((uint8_t*)"<!> Baud Rate Write Ok -> Restart now"); 
                    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                    HAL_NVIC_SystemReset(); 
                  }
                  else 
                  {
                    Broker_Publish_Topic((uint8_t*)"<X> Baud Rate Write Error");
                    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                  }
                }
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Invalid Baud Rate Syntax"); 
				      Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }

            Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            break; 

          //...If commands then clear 
          case SERVERCMD_GOTOCFG_RS485:
          case SERVERCMD_GOTOCFG_INTERVAL:
          case SERVERCMD_GOTOCFG_BAUDRATE:
          case SERVERCMD_GOTOCFG_TIMEC2C:
          case SERVERCMD_WRITECFG_RS485:
          case SERVERCMD_WRITECFG_INTERVAL:
          case SERVERCMD_WRITECFG_TIMEC2C:
          case SERVERCMD_RESTART:
          case SERVERCMD_BREAK:
						Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						break; 
        }

        HAL_Delay(1000); 
        Timeout_UserConfig--; 
        if(Timeout_UserConfig == 0)
        {
          if(Eeprom_Read_Data(Addr_Baudrate, Position_1) == 0xFFFFFFFF) 
          {
            //...Clear page 
            Eeprom_ClearPage(User_ConfigBaudrate_Page);

            //...Check write data 
            if(Eeprom_Write_Data(Addr_Baudrate, Position_1, Baudrate_Default, 1000) == FLASH_OK) 
            {
              if(Eeprom_Read_Data(Addr_Baudrate, Position_1) != 0xFFFFFFFF)
              {
                Broker_Publish_Topic((uint8_t*)"<!> Baud Rate Default Write Ok"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Baud Rate Default Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Baud Rate Already exists"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }
          }
          else 
          {
            Broker_Publish_Topic((uint8_t*)"<!> Timeout!"); 
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
          }

          Mode_Flag = SYSMONITOR_NORMAL_MODE;
          break; 
        }
      }
    }
    else if(Mode_Flag == SYSMONITOR_CONFIGRS485_MODE)
    {
      sprintf((char*)Filter_Buf, "%s", "<!> Started config modbus RTU"); 
      Broker_Publish_Topic(Filter_Buf); 
			Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
			Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
			HAL_Delay(100); 

      Eeprom_ClearPage(User_ConfigRs485Cmd_Page); 
      Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
      Timeout_UserConfig = Set_Time; 
      while(1)
      {
        DATA_DISPLAY = Config_Mode; 
        uint8_t cmd = Configuration_Commands_Handle(IoT_Protocol_Buf, IoT_Byte); 

        //...Wait for the end of string
				while(cmd != NONE_CMD)
        {
          if(__HAL_TIM_GetCounter(&htim6) > 5)
          {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Stop(&htim6);

            break; 
          }
        } 

        switch(cmd)
        {
          case SERVERCMD_WRITECFG_RS485:
            //...+6 Remove string "RS485-" 
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "RS485") + 6; 

            //...Filter string 
						for(uint8_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0' || Analysis[i] == 0) break; 
							else Filter_Buf[i] = Analysis[i];						
						}
						Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
					
						HexStrToHexChar(Filter_Buf, Send_Package_Buf); 
						CRC_Byte = ModbusRTU_Calc_CRC16(Send_Package_Buf, strlen((char*)Filter_Buf) / 2); 
						if(((strlen((char*)Filter_Buf) % 2) == 0) && CRC_Byte) 
						{
							if(CmdConfig_Cnt < 50)
							{
								CmdConfig_Cnt++; 	
								if((Eeprom_Write_Data(Addr_Size_Rs485 + Config_Addr_Next, Position_1, CRC_Byte, 1000) | Eeprom_WriteModbus_Rtu(Addr_Data_Rs485 + Config_Addr_Next, Position_2, (uint64_t*)Send_Package_Buf, 1000)) == FLASH_OK)
								{
									Config_Addr_Next += 16; 
									Broker_Publish_Topic((uint8_t*)"<!> RS485 Commands Write Ok"); 
                  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
								}
								else 
                {
                  Broker_Publish_Topic((uint8_t*)"<X> RS485 Commands Write Error"); 
                  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
                }
							}
							else 
							{
								Broker_Publish_Topic((uint8_t*)"<X> Memory Low"); 
								Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
								
								Stm32_Clear_Buf(Send_Package_Buf, Send_Byte);
								Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 								
								Timeout_UserConfig = Set_Time;
								break;  
							}
						} 
						else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Invalid Checksum");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
						
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
						Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            Timeout_UserConfig = Set_Time;
            break; 
    
          case SERVERCMD_BREAK:
						if(Eeprom_Write_Data(Addr_CmdConfig, Position_1, CmdConfig_Cnt, 1000) == FLASH_OK)
						{
							Broker_Publish_Topic((uint8_t*)"<!> Stopped configure -> Restart now");
							Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
							HAL_NVIC_SystemReset(); 
						}
						else 
						{
							Broker_Publish_Topic((uint8_t*)"<X> Write Error"); 
							Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
							Timeout_UserConfig = Set_Time;
						}
            break; 
					
          //...If commands then clear 
          case SERVERCMD_GOTOCFG_RS485:
          case SERVERCMD_GOTOCFG_INTERVAL:
          case SERVERCMD_GOTOCFG_BAUDRATE:
          case SERVERCMD_GOTOCFG_TIMEC2C:
          case SERVERCMD_WRITECFG_INTERVAL:
          case SERVERCMD_WRITECFG_BAUDRATE:
          case SERVERCMD_WRITECFG_TIMEC2C:
          case SERVERCMD_RESTART:
						Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
						break; 
        }
        
        HAL_Delay(1000); 
        Timeout_UserConfig--; 
        if(Timeout_UserConfig == 0)
        {
          if(Eeprom_Read_Data(Addr_CmdConfig, Position_1) == 0xFFFFFFFF) 
          {
            if(Eeprom_Write_Data(Addr_CmdConfig, Position_1, CmdConfig_Cnt, 1000) == FLASH_OK)
            {
              if(Eeprom_Read_Data(Addr_CmdConfig, Position_1) != 0xFFFFFFFF) 
              {
                Broker_Publish_Topic((uint8_t*)"<!> Amount RS485 Commands Write Ok");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
              }
              else
              {
                Broker_Publish_Topic((uint8_t*)"<!> Amount RS485 Commands Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> RS485 Commands Already exists"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }
          }

          if(Eeprom_Read_Data(Addr_Interval, Position_1) == 0xFFFFFFFF) 
          {
            //...Value Interval Default
            const uint8_t Interval_Default = 45;

            //...Clear page 
            Eeprom_ClearPage(User_ConfigInterval_Page); 

            //...Check write data 
            if(Eeprom_Write_Data(Addr_Interval, Position_1, Interval_Default, 1000) == FLASH_OK) 
            {
              if(Eeprom_Read_Data(Addr_Interval, Position_1) != 0xFFFFFFFF) 
              {
                Broker_Publish_Topic((uint8_t*)"<!> Interval Default Write Ok"); 
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
              else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Interval Default Write Error");
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
              }
            }
            else 
            {
              Broker_Publish_Topic((uint8_t*)"<X> Interval Already exists"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
          }

          Broker_Publish_Topic((uint8_t*)"<!> Timeout!"); 
          Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
          Mode_Flag = SYSMONITOR_NORMAL_MODE; 
          break; 
        }
      }
    }

    if(Mode_Flag == SYSMONITOR_NORMAL_MODE)
    {
      const uint8_t TimeCmd2Cmd_Default = 200;  
			if(Eeprom_Read_Data(Addr_C2C, Position_2) == 0xFFFFFFFF) Eeprom_Write_Data(Addr_C2C, Position_2, TimeCmd2Cmd_Default, 1000); 
			
      #ifdef CBSC
      if(Eeprom_Read_Data(Addr_CmdConfig, Position_1) == 0xFFFFFFFF || Eeprom_Read_Data(Addr_CmdConfig, Position_1) == 0x00000000) CmdConfig_Cnt = 0;
			else CmdConfig_Cnt = Eeprom_Read_Data(Addr_CmdConfig, Position_1); 
      #else
      if(Eeprom_Read_Data(Addr_CmdConfig, Position_1) == 0xFFFFFFFF || Eeprom_Read_Data(Addr_CmdConfig, Position_1) == 0x00000000) 
      {
        CmdConfig_Cnt = 0;  

        //...Clear page 
        Eeprom_ClearPage(User_ConfigInterval_Page); 

        //...Write Interval into Eeprom 
        Eeprom_Write_Data(Addr_Interval, Position_1, 0, 1000); 
      }
			else CmdConfig_Cnt = Eeprom_Read_Data(Addr_CmdConfig, Position_1); 
      #endif

      sprintf((char*)Send_Package_Buf, "<!> Started data -> Device %s --%s --RSSI(dBm): %d --Commands config: %d --Interval(s): %ld", DeviceID, Version, Get_RSSI_Sim4GLTE(huart2, IoT_Protocol_Buf), CmdConfig_Cnt, Eeprom_Read_Data(Addr_Interval, Position_1));
			Broker_Publish_Topic(Send_Package_Buf);
			Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
			Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
      HAL_Delay(500);  
			
			uint8_t CmdConfig_Amount = 0, Config_Addr_Position = 0, IT_Handle = 0; 
      uint32_t Eep_Read = 0; 
      while(1)
      { 
        #ifdef CBSC
          if(CheckInput())
          {
            intervalValue = 10; // chuyen interval thanh 10s
          }
          else intervalValue = Eeprom_Read_Data(Addr_Interval, Position_1); //get data interval from eprom
          
          if(HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET) 
          {
            HAL_Delay(50); 

            while(HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET)
            {
              if(HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_SET) 
              {
                Broker_Publish_Topic((uint8_t*)"BUTTO-1"); 
                break; 
              }
            }
          }
        #endif
        DATA_DISPLAY = Normal_Mode; 
				
				if(CmdConfig_Cnt == 0) 
				{
					if(Req_Ping_Flag == 1) 
					{
            Broker_Publish_Topic((uint8_t*)"REQUEST_PING");
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
						Req_Ping_Flag = 0; 
					}
				}
				
        uint8_t cmd = Normal_CommandsHandle(IoT_Protocol_Buf, IoT_Byte); 

        //...Wait for the end of string 
        while(cmd != NONE_CMD)
        {
          if(__HAL_TIM_GetCounter(&htim6) > 5)
          {
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Stop(&htim6);

            break; 
          }
        } 

        switch(cmd)
        {
          case SERVERCMD_NORMAL_RESTART: Stm32_Restart(); break;
          case SERVERCMD_NORMAL_RS485: 
						Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 

            //...+6 Remove string "RS485-"
            Analysis = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "RS485") + 6; 
            for(uint16_t i = 0; i < Filter_Byte; i++)
						{
							if(Analysis[i] == 0x0D || Analysis[i] == 0x0A || Analysis[i] == '\0') break; 
							else Filter_Buf[i] = Analysis[i];						
						}

            Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 
						Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
						
            HexStrToHexChar(Filter_Buf, Send_Package_Buf); 
            CRC_Byte = ModbusRTU_Calc_CRC16(Send_Package_Buf, strlen((char*)Filter_Buf) / 2);
						
            if(((strlen((char*)Filter_Buf) % 2) == 0) && CRC_Byte)
            {
							DATA_DISPLAY = Data_Processing;

              //...Sending data to bus 
              //HAL_GPIO_WritePin(Output_Modbus_Control_GPIO_Port, Output_Modbus_Control_Pin, GPIO_PIN_SET);
							//Relay_Control(Relay_GPIO_Port, Relay_Pin, HIGH, ON);
							//HAL_Delay(250); 

							HAL_UART_Transmit(&huart1, Send_Package_Buf, strlen((char*)Filter_Buf) / 2, 1000);

              //...Getting data from bus
							//HAL_GPIO_WritePin(Output_Modbus_Control_GPIO_Port, Output_Modbus_Control_Pin, GPIO_PIN_RESET);
              Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 

              //...Delay receive data from Inverter 
              uint16_t timeout = 300; 
              while (1)
              {
                if(__HAL_TIM_GetCounter(&htim15) > 5)
                {
                  __HAL_TIM_SET_COUNTER(&htim15, 0);
                  HAL_TIM_Base_Stop(&htim15);

                  break;  
                }

                HAL_Delay(1);
                timeout--; 
                if(timeout == 0) break; 
              }
							
							if(ModbusRTU_Check_RecvData(Inverter_Buf, Inverter_Byte, 1000) != 0)
              {
								Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 
								Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 

                if(ModbusRTU_Calc_CRC16(Inverter_Buf, Inverter_Count) != 0) 
								{
									HexCharToHexStr(Inverter_Buf, Inverter_Count, Receive_Package_Buf); 			
									sprintf((char*)Send_Package_Buf, "RS485-%s", Receive_Package_Buf); 
									Broker_Publish_Topic(Send_Package_Buf);
								}
								else 
                {
                  Broker_Publish_Topic((uint8_t*)"<X> Invalid data");		
                  Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte);
                }
              }
              else Broker_Publish_Topic((uint8_t*)"<X> No data"); 
            }
            else Broker_Publish_Topic((uint8_t*)"<X> Invalid Checksum"); 
            
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 

            //Relay_Control(Relay_GPIO_Port, Relay_Pin, HIGH, OFF); 
						Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte);
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte);  
						Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 
            Stm32_Clear_Buf(Filter_Buf, Filter_Byte); 
            break; 
          
          case SERVERCMD_NORMAL_QUEST_CMD:
						Config_Addr_Next = 0, CmdConfig_Amount = 0, Config_Addr_Position = 0; 

            Eep_Read = Eeprom_Read_Data(Addr_CmdConfig, Position_1); 
						if(Eep_Read == 0xFFFFFFFF || Eep_Read == 0x00000000) Eep_Read = 0; 
						if(Eep_Read != 0)
						{
							for(uint8_t x = 0; x < Eep_Read; x++) 
							{
								CmdConfig_Amount = Eeprom_Read_Data(Addr_Size_Rs485 + Config_Addr_Next, Position_1);
								Config_Addr_Position = 8; 
								
								for(uint8_t i = 0; i < Eeprom_Byte; i++) 
								{
									Eeprom_Buf[i] = Eeprom_ReadModbus_Rtu(Addr_Data_Rs485 + Config_Addr_Next + Config_Addr_Position); 
									Config_Addr_Position += 4; 
								}
								
								Config_Addr_Next += 16; 
								Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte);  
								HexCharToHexStr((uint8_t*)Eeprom_Buf, CmdConfig_Amount, Receive_Package_Buf);
								
								sprintf((char*)Send_Package_Buf, "%s%c%s", "RS485", '-', Receive_Package_Buf); 
								Broker_Publish_Topic(Send_Package_Buf);
                Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 

								Stm32_Clear_Buf(Send_Package_Buf, Send_Byte);
                Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 
								Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte);
							}
						}
            else 
						{
							Broker_Publish_Topic((uint8_t*)"<X> No RS485 Commands");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
						}
            break;
          
          case SERVERCMD_NORMAL_PING: 
            #ifdef CBSC
            sendCBSC();
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            break;
            #else
            sprintf((char*)Send_Package_Buf, "%s%ld%s%d", "PING	(*)INTERVAL: ", Eeprom_Read_Data(Addr_Interval, Position_1), "	(*)RSSI: ", Get_RSSI_Sim4GLTE(huart2, IoT_Protocol_Buf)); 
            Broker_Publish_Topic(Send_Package_Buf); 
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
          
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            break; 
            #endif
					case SERVERCMD_NORMAL_CONFIRM:
						Broker_Publish_Topic((uint8_t*)"<!> Confirm!");
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
					
						Interval_Flag = 1; 
						break; 

          case SERVERCMD_NORMAL_SIM_RSSI:
            sprintf((char*)Send_Package_Buf, "RSSI: %d dBm - Network provider: ", Get_RSSI_Sim4GLTE(huart2, IoT_Protocol_Buf)); 
            Get_Service_Provider_Name_Sim4GLTE(huart2, IoT_Protocol_Buf, Send_Package_Buf); 

            Broker_Publish_Topic(Send_Package_Buf);
            Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
            break; 

          case SERVERCMD_NORMAL_WIRELESS:
            if(wirelessProtocol == SimA7670C_Modules)
            {
              Broker_Publish_Topic((uint8_t*)">> The device is using 4G-LTE Network"); 
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
            }
            else if(wirelessProtocol == Esp32_Modules)
            {
              Broker_Publish_Topic((uint8_t*)">> The device is using Internet WiFi");
              Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
            }
            break;
        }
				
        //...Device query to Inverter with interval
        if(Interval_Flag)
        {
					HAL_TIM_Base_Stop_IT(&htim3); 

					CmdConfig_Amount = 0, IT_Handle = 0; 
				
          #ifdef CBSC
            sendCBSC();
          #else

          uint8_t Eep_Addr_Position = 0; 
					uint16_t Eep_Addr_Next = 0, Time_Send = 0; 

          for(uint8_t x = 0; x < CmdConfig_Cnt; x++) 
					{
						DATA_DISPLAY = Data_Processing; 

            //...Sending data to bus
						//HAL_GPIO_WritePin(Output_Modbus_Control_GPIO_Port, Output_Modbus_Control_Pin, GPIO_PIN_SET);
						Relay_Control(Relay_GPIO_Port, Relay_Pin, HIGH, ON);		
						HAL_Delay(250);
						
						CmdConfig_Amount = Eeprom_Read_Data(Addr_Size_Rs485 + Eep_Addr_Next, Position_1);
						Eep_Addr_Position = 8; 
						
						Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte);						
						Stm32_Clear_Buf(Send_Package_Buf, Send_Byte);
						Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte);
						Stm32_Clear_Buf((uint8_t*)Eeprom_Buf, Eeprom_Byte); 
						
						for(uint8_t i = 0; i < Eeprom_Byte; i++) 
						{
							Eeprom_Buf[i] = Eeprom_ReadModbus_Rtu(Addr_Data_Rs485 + Eep_Addr_Next + Eep_Addr_Position);
							Eep_Addr_Position += 4; 
						}
						Eep_Addr_Next += 16; 
						
						if(ModbusRTU_Calc_CRC16((uint8_t*)Eeprom_Buf, CmdConfig_Amount) == CmdConfig_Amount)
						{
							HAL_UART_Transmit(&huart1, (uint8_t*)Eeprom_Buf, CmdConfig_Amount, 1000);

              //...Getting data from bus
							//HAL_GPIO_WritePin(Output_Modbus_Control_GPIO_Port, Output_Modbus_Control_Pin, GPIO_PIN_RESET);
              Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 
              
              //...Delay receive data from Inverter  
              uint16_t timeout = 300; 
              while (1)
              {
                if(__HAL_TIM_GetCounter(&htim15) > 10)
                {
                  __HAL_TIM_SET_COUNTER(&htim15, 0);
                  HAL_TIM_Base_Stop(&htim15);

                  break;  
                }

                HAL_Delay(1);
                timeout--; 
                if(timeout == 0) break; 
              }
						}
						else 
						{
							Broker_Publish_Topic((uint8_t*)"<X> Invalid Eeprom");
							Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
							break; 
						}

						if(ModbusRTU_Check_RecvData(Inverter_Buf, Inverter_Byte, 1000) != 0)
						{
							Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 
							Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
								
              if(ModbusRTU_Calc_CRC16(Inverter_Buf, Inverter_Count) != 0)
							{
								HexCharToHexStr(Inverter_Buf, Inverter_Count, Receive_Package_Buf); 			
								sprintf((char*)Send_Package_Buf, "RS485-%d-%s", x, Receive_Package_Buf);  
                Broker_Publish_Topic(Send_Package_Buf);	
							}
							else 
              {
                Broker_Publish_Topic((uint8_t*)"<X> Invalid data");		
                Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 
              } 
						}
						else Broker_Publish_Topic((uint8_t*)"<X> No data"); 
						
						IT_Handle = Normal_CommandsHandle(IoT_Protocol_Buf, IoT_Byte); 
						if(IT_Handle != 0) break;
						else 
						{
							Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
							
              //...Unit Milliseconds
							Time_Send = Eeprom_Read_Data(Addr_C2C, Position_2); 
							if(Time_Send < 200) Time_Send = 200; 
							HAL_Delay(Time_Send);
						} 
					}
          #endif
					
					Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 
					Stm32_Clear_Buf(Send_Package_Buf, Send_Byte); 
					Stm32_Clear_Buf(Receive_Package_Buf, Receive_Byte); 
					Stm32_Clear_Buf((uint8_t*)Eeprom_Buf, Eeprom_Byte);
					if(IT_Handle != SERVERCMD_NORMAL_RS485) 
					{
						Relay_Control(Relay_GPIO_Port, Relay_Pin, HIGH, OFF);
						HAL_Delay(250); 
					}
					
					HAL_TIM_Base_Start_IT(&htim3); 
					Interval_Flag = 0; 
        }
				else 
				{
          #ifdef CBSC
          if(strstr((char*)IoT_Protocol_Buf, "SIM REMOVED")) HAL_NVIC_SystemReset();
          #else 
					if(strstr((char*)IoT_Protocol_Buf, "ERROR") || strstr((char*)IoT_Protocol_Buf, "SEND FAIL") || strstr((char*)IoT_Protocol_Buf, "CLOSED")) HAL_NVIC_SystemReset();
          #endif 
        }

        Stm32_Clear_Buf(Inverter_Buf, Inverter_Byte); 
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 16000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 9;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PhysRestart_Sim7020_GPIO_Port, PhysRestart_Sim7020_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_LED_Pin|Battery_LED_Pin|Test_LED_Pin|SOS_LED_Pin
                          |PhysRestart_Pin|IO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Relay_Pin */
  GPIO_InitStruct.Pin = Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Relay_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Checking_Esp32_Pin */
  GPIO_InitStruct.Pin = Checking_Esp32_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Checking_Esp32_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PhysRestart_Sim7020_Pin */
  GPIO_InitStruct.Pin = PhysRestart_Sim7020_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PhysRestart_Sim7020_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_LED_Pin Battery_LED_Pin Test_LED_Pin SOS_LED_Pin
                           PhysRestart_Pin IO2_Pin */
  GPIO_InitStruct.Pin = DC_LED_Pin|Battery_LED_Pin|Test_LED_Pin|SOS_LED_Pin
                          |PhysRestart_Pin|IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Input_Device_Pin WiFi_Reset_Pin */
  GPIO_InitStruct.Pin = Input_Device_Pin|WiFi_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
ServerCmd_ConfigModHandleTypeDef Configuration_Commands_Handle(uint8_t *recv_buf, uint16_t size)
{	
  for(uint16_t i = 0; i < size; i++)
  {
    //...cmd 'ID''
    if(recv_buf[i] == 'I' && recv_buf[i + 1] == 'D') return SERVERCMD_WRITECFG_ID; 

    //...cmd 'HOST'
    else if(recv_buf[i] == 'H' && recv_buf[i + 1] == 'O' && recv_buf[i + 2] == 'S' && recv_buf[i + 3] == 'T') return SERVERCMD_WRITECFG_HOST; 

    //...cmd 'PORT'
    else if(recv_buf[i] == 'P' && recv_buf[i + 1] == 'O' && recv_buf[i + 2] == 'R' && recv_buf[i + 3] == 'T') return SERVERCMD_WRITECFG_PORT; 

    //...cmd 'BREAK'
    else if(recv_buf[i] == 'B' && recv_buf[i + 1] == 'R' && recv_buf[i + 2] == 'E' && recv_buf[i + 3] == 'A' && recv_buf[i + 4] == 'K') return SERVERCMD_BREAK; 

    //...cmd 'RS485'
    else if(recv_buf[i] == 'R' && recv_buf[i + 1] == 'S' && recv_buf[i + 2] == '4' && recv_buf[i + 3] == '8' && recv_buf[i + 4] == '5') return SERVERCMD_WRITECFG_RS485; 
    
    //...cmd 'TIMEC2C'
    else if(recv_buf[i] == 'T' && recv_buf[i + 1] == 'I' && recv_buf[i + 2] == 'M' && recv_buf[i + 3] == 'E' && recv_buf[i + 4] == 'C' && recv_buf[i + 5] == '2' && recv_buf[i + 6] == 'C') return SERVERCMD_WRITECFG_TIMEC2C; 
    
    //...cmd 'RESTART'
    else if(recv_buf[i] == 'R' && recv_buf[i + 1] == 'E' && recv_buf[i + 2] == 'S' && recv_buf[i + 3] == 'T' && recv_buf[i + 4] == 'A' && recv_buf[i + 5] == 'R' && recv_buf[i + 6] == 'T') return SERVERCMD_RESTART; 
    
    //...cmd 'INTERVAL'
    else if(recv_buf[i] == 'I' && recv_buf[i + 1] == 'N' && recv_buf[i + 2] == 'T' && recv_buf[i + 3] == 'E' && recv_buf[i + 4] == 'R' && recv_buf[i + 5] == 'V' && recv_buf[i + 6] == 'A' && recv_buf[i + 7] == 'L') return SERVERCMD_WRITECFG_INTERVAL; 
    
    //...cmd 'BAUDRATE'
    else if(recv_buf[i] == 'B' && recv_buf[i + 1] == 'A' && recv_buf[i + 2] == 'U' && recv_buf[i + 3] == 'D' && recv_buf[i + 4] == 'R' && recv_buf[i + 5] == 'A' && recv_buf[i + 6] == 'T' && recv_buf[i + 7] == 'E') return SERVERCMD_WRITECFG_BAUDRATE; 

    //...cmd 'USERNAME'
    else if(recv_buf[i] == 'U' && recv_buf[i + 1] == 'S' && recv_buf[i + 2] == 'E' && recv_buf[i + 3] == 'R' && recv_buf[i + 4] == 'N' && recv_buf[i + 5] == 'A' && recv_buf[i + 6] == 'M' && recv_buf[i + 7] == 'E') return SERVERCMD_WRITECFG_USERNAME; 
    
    //...cmd 'PASSWORD'
    else if(recv_buf[i] == 'P' && recv_buf[i + 1] == 'A' && recv_buf[i + 2] == 'S' && recv_buf[i + 3] == 'S' && recv_buf[i + 4] == 'W' && recv_buf[i + 5] == 'O' && recv_buf[i + 6] == 'R' && recv_buf[i + 7] == 'D') return SERVERCMD_WRITECFG_PASSWORD; 
    
    //...cmd 'TOPICSUB'
    else if(recv_buf[i] == 'T' && recv_buf[i + 1] == 'O' && recv_buf[i + 2] == 'P' && recv_buf[i + 3] == 'I' && recv_buf[i + 4] == 'C' && recv_buf[i + 5] == 'S' && recv_buf[i + 6] == 'U' && recv_buf[i + 7] == 'B') return SERVERCMD_WRITECFG_TOPICSUB; 
    
    //...cmd 'TOPICPUB'
    else if(recv_buf[i] == 'T' && recv_buf[i + 1] == 'O' && recv_buf[i + 2] == 'P' && recv_buf[i + 3] == 'I' && recv_buf[i + 4] == 'C' && recv_buf[i + 5] == 'P' && recv_buf[i + 6] == 'U' && recv_buf[i + 7] == 'B') return SERVERCMD_WRITECFG_TOPICPUB; 
    
    //...cmd 'CFG_RS485'
    else if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'F' && recv_buf[i + 2] == 'G' && recv_buf[i + 3] == '_' && recv_buf[i + 4] == 'R' && recv_buf[i + 5] == 'S' && recv_buf[i + 6] == '4' && recv_buf[i + 7] == '8' && recv_buf[i + 8] == '5') return SERVERCMD_GOTOCFG_RS485; 

    //...cmd 'CFG_BROKER'
    else if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'F' && recv_buf[i + 2] == 'G' && recv_buf[i + 3] == '_' && recv_buf[i + 4] == 'B' && recv_buf[i + 5] == 'R' && recv_buf[i + 6] == 'O' && recv_buf[i + 7] == 'K' && recv_buf[i + 8] == 'E' && recv_buf[i + 9] == 'R') return SERVERCMD_GOTOCFG_BROKER; 
    
    //...cmd 'CFG_TIMC2C'
    else if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'F' && recv_buf[i + 2] == 'G' && recv_buf[i + 3] == '_' && recv_buf[i + 4] == 'T' && recv_buf[i + 5] == 'I' && recv_buf[i + 6] == 'M' && recv_buf[i + 7] == 'E' && recv_buf[i + 8] == 'C' && recv_buf[i + 9] == '2' && recv_buf[i + 10] == 'C') return SERVERCMD_GOTOCFG_TIMEC2C; 
    
    //...cmd 'CFG_INTERVAL'
    else if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'F' && recv_buf[i + 2] == 'G' && recv_buf[i + 3] == '_' && recv_buf[i + 4] == 'I' && recv_buf[i + 5] == 'N' && recv_buf[i + 6] == 'T' && recv_buf[i + 7] == 'E' && recv_buf[i + 8] == 'R' && recv_buf[i + 9] == 'V' && recv_buf[i + 10] == 'A' && recv_buf[i + 11] == 'L') return SERVERCMD_GOTOCFG_INTERVAL; 
    
    //...cmd 'CFG_BAUDRATE'
    else if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'F' && recv_buf[i + 2] == 'G' && recv_buf[i + 3] == '_' && recv_buf[i + 4] == 'B' && recv_buf[i + 5] == 'A' && recv_buf[i + 6] == 'U' && recv_buf[i + 7] == 'D' && recv_buf[i + 8] == 'R' && recv_buf[i + 9] == 'A' && recv_buf[i + 10] == 'T' && recv_buf[i + 11] == 'E') return SERVERCMD_GOTOCFG_BAUDRATE; 
  }
	
	return NONE_CONFIG;
}

ServerCmd_NormalModHandleTypeDef Normal_CommandsHandle(uint8_t *recv_buf, uint16_t size)
{
  for(uint16_t i = 0; i < size; i++)
  {
    //...cmd 'CMD?'
    if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'M' && recv_buf[i + 2] == 'D' && recv_buf[i + 3] == '?') return SERVERCMD_NORMAL_QUEST_CMD; 
    
    //...cmd 'PING'
    else if(recv_buf[i] == 'P' && recv_buf[i + 1] == 'I' && recv_buf[i + 2] == 'N' && recv_buf[i + 3] == 'G') return SERVERCMD_NORMAL_PING; 
    
    //...cmd 'RS485'
    else if(recv_buf[i] == 'R' && recv_buf[i + 1] == 'S' && recv_buf[i + 2] == '4' && recv_buf[i + 3] == '8' && recv_buf[i + 4] == '5') return SERVERCMD_NORMAL_RS485; 
    
    //...cmd 'RESTART'
    else if(recv_buf[i] == 'R' && recv_buf[i + 1] == 'E' && recv_buf[i + 2] == 'S' && recv_buf[i + 3] == 'T' && recv_buf[i + 4] == 'A' && recv_buf[i + 5] == 'R' && recv_buf[i + 6] == 'T') return SERVERCMD_NORMAL_RESTART; 
    
    //...cmd 'RELAY-1'
    else if(recv_buf[i] == 'R' && recv_buf[i + 1] == 'E' && recv_buf[i + 2] == 'L' && recv_buf[i + 3] == 'A' && recv_buf[i + 4] == 'Y' && recv_buf[i + 5] == '-' && recv_buf[i + 6] == '1') return SERVERCMD_NORMAL_RELAY_1; 
    
    //...cmd 'REALY-0'
    else if(recv_buf[i] == 'R' && recv_buf[i + 1] == 'E' && recv_buf[i + 2] == 'L' && recv_buf[i + 3] == 'A' && recv_buf[i + 4] == 'Y' && recv_buf[i + 5] == '-' && recv_buf[i + 6] == '0') return SERVERCMD_NORMAL_RELAY_0; 
    
    //...cmd 'CONFIRM'
    else if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'O' && recv_buf[i + 2] == 'N' && recv_buf[i + 3] == 'F' && recv_buf[i + 4] == 'I' && recv_buf[i + 5] == 'R' && recv_buf[i + 6] == 'M') return SERVERCMD_NORMAL_CONFIRM; 

    //...cmd 'SIM_RSSI?'
    else if(recv_buf[i] == 'S' && recv_buf[i + 1] == 'I' && recv_buf[i + 2] == 'M' && recv_buf[i + 3] == '_' && recv_buf[i + 4] == 'R' && recv_buf[i + 5] == 'S' && recv_buf[i + 6] == 'S' && recv_buf[i + 7] == 'I' && recv_buf[i + 8] == '?') return SERVERCMD_NORMAL_SIM_RSSI; 

    //...cmd 'WIRELESS?' 
    else if(recv_buf[i] == 'W' && recv_buf[i + 1] == 'I' && recv_buf[i + 2] == 'R' && recv_buf[i + 3] == 'E' && recv_buf[i + 4] == 'L' && recv_buf[i + 5] == 'E' && recv_buf[i + 6] == 'S' && recv_buf[i + 7] == 'S' && recv_buf[i + 8] == '?') return SERVERCMD_NORMAL_WIRELESS; 
  }

  return NONE_CMD; 
}

Syntax_HandleTypeDef Is_Setup_Right(uint8_t *recv_buf, uint16_t size)
{
	uint8_t *Check_Content = 0; 
  uint8_t tempo[100]={0}; 
	
  for(uint16_t i = 0; i < size; i++)
  {
    if(recv_buf[i] == 'I' && recv_buf[i + 1] == 'N' && recv_buf[i + 2] == 'T' && recv_buf[i + 3] == 'E' && recv_buf[i + 4] == 'R' && recv_buf[i + 5] == 'V' && recv_buf[i + 6] == 'A' && recv_buf[i + 7] == 'L')
    {
      if(recv_buf[i + 8] == '=')
      {
        //+9: Remove string "INTERVAL=" 
				Check_Content = (uint8_t*)strstr((char*)recv_buf, "INTERVAL") + 9; 

        if(wirelessProtocol == Esp32_Modules) 
        {
          for(uint8_t j = 0; j < strlen((char*)Check_Content) - 2; j++) if((Check_Content[j] < 48) || (Check_Content[j] > 57)) return INVALID_SYNTAX; 
        }
        else if(wirelessProtocol == SimA7670C_Modules) 
        {
          for(uint8_t i = 0; i < 100; i++)
          {
            if(Check_Content[i] == 0x0D || Check_Content[i] == 0x0A || Check_Content[i] == '+') break; 
            else tempo[i] = Check_Content[i];						

            for(uint8_t j = 0; j < strlen((char*)tempo); j++) if((tempo[j] < 48) || (tempo[j] > 57)) return INVALID_SYNTAX; 
          }
        }
      } 
      else return INVALID_SYNTAX; 
    }

    if(recv_buf[i] == 'B' && recv_buf[i + 1] == 'A' && recv_buf[i + 2] == 'U' && recv_buf[i + 3] == 'D' && recv_buf[i + 4] == 'R' && recv_buf[i + 5] == 'A' && recv_buf[i + 6] == 'T' && recv_buf[i + 7] == 'E') 
    {
      if(recv_buf[i + 8] == '=')
      {
        //+9: Remove string "BAUDRATE=" 
        Check_Content = (uint8_t*)strstr((char*)recv_buf, "BAUDRATE") + 9;  
        for(uint8_t j = 0; j < strlen((char*)Check_Content) - 2; j++) if((Check_Content[j] < 48) || (Check_Content[j] > 57)) return INVALID_SYNTAX; 
      }
      else return INVALID_SYNTAX; 
    }
    
    if(recv_buf[i] == 'C' && recv_buf[i + 1] == 'M' && recv_buf[i + 2] == 'D' && recv_buf[i + 3] == 'S' && recv_buf[i + 4] == 'E' && recv_buf[i + 5] == 'N' && recv_buf[i + 6] == 'D') 
    {
      if(recv_buf[i + 7] == '=') 
      {
        //+8: Remove string "CMDSEND="
				Check_Content = (uint8_t*)strstr((char*)recv_buf, "CMDSEND")+ 8; 
				for(uint8_t j = 0; j < strlen((char*)Check_Content) - 2; j++) if((Check_Content[j] < 48) || (Check_Content[j] > 57)) return INVALID_SYNTAX;
			}
      else return INVALID_SYNTAX;
    }

    if(recv_buf[i] == 'P' && recv_buf[i + 1] == 'O' && recv_buf[i + 2] == 'R' && recv_buf[i + 3] == 'T')
    {
      if(recv_buf[i + 4] == ':')
      {
        //+5: Remove string "PORT:"
        Check_Content = (uint8_t*)strstr((char*)recv_buf, "PORT") + 5; 
        for(uint8_t j = 0; j < strlen((char*)Check_Content) - 2; j++) if((Check_Content[j] < 48) || (Check_Content[j] > 57)) return INVALID_SYNTAX; 
      }
    }

    if(recv_buf[i] == 'I' && recv_buf[i + 1] == 'D')
    {
      if(recv_buf[i + 2] == ':')
      {
        //+3: Remove string "ID:"
        Check_Content = (uint8_t*)strstr((char*)recv_buf, "ID") + 3; 
        for(uint8_t j = 0; j < strlen((char*)Check_Content) - 2; j++) if((Check_Content[j] < 48) || (Check_Content[j] > 57)) return INVALID_SYNTAX; 
      }
    }
  }
  return CORRECT_SYNTAX; 
}

SysMonitor_ModeHandleTypeDef Waiting_For_Config(uint8_t *recv_buf, uint16_t size, uint16_t timeout) 
{
  //...Delay receive data from Esp8266 
  while (1)
  {
    if(__HAL_TIM_GetCounter(&htim6) > 5)
    {
      __HAL_TIM_SET_COUNTER(&htim6, 0);
      HAL_TIM_Base_Stop(&htim6);

      uint8_t cmd = Configuration_Commands_Handle(recv_buf, size);

      if(cmd == SERVERCMD_GOTOCFG_RS485) return SYSMONITOR_CONFIGRS485_MODE; 
      else if(cmd == SERVERCMD_GOTOCFG_INTERVAL) return SYSMONITOR_CONFIGINTERVAL_MODE; 
      else if(cmd == SERVERCMD_GOTOCFG_BAUDRATE) return SYSMONITOR_CONFIGBAUDRATE_MODE; 
      else if(cmd == SERVERCMD_GOTOCFG_TIMEC2C) return SYSMONITOR_CONFIGCMD2CMD_MODE; 
      else if(cmd == SERVERCMD_GOTOCFG_BROKER) return SYSMONITOR_CONFIGBROKER_MODE; 
      else if(cmd == SERVERCMD_RESTART) Stm32_Restart(); 
    }

    HAL_Delay(1);
    timeout--; 
    if(timeout == 0) return SYSMONITOR_NORMAL_MODE;
  }

}

WiFi_StatusTypeDef Stm32_Check_Esp32_Connect_WiFi(uint16_t timeout)
{
	while(1)
	{
    WIRELESS_DISPLAY = Check_Connect_WiFi; 
		Esp32_Check_Info_WiFi(huart3); 
		uint8_t *_pStr = (uint8_t*)strstr((char*)IoT_Protocol_Buf, "+CWJAP"); 
    
		if(_pStr != 0)
		{
			HAL_Delay(100); 
			if(Esp32_Response(IoT_Protocol_Buf, (uint8_t*)"OK", 10000)) Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
			return WIFI_CONNECTED; 
		}		
		
		HAL_Delay(2000); 
		timeout--; 
		if(timeout == 0) {
			HAL_Delay(100); 
			if(Esp32_Response(IoT_Protocol_Buf, (uint8_t*)"OK", 10000)) Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte);  
			return WIFI_DISCONNECTED;
		}			
	}
}

SimCard_StatusTypeDef Stm32_Check_Sim_Card_Connect_Network_Service(uint16_t timeout)
{
  while(1)
  {
    WIRELESS_DISPLAY = Check_Insert_Sim_Card; 
    Is_Insert_Sim_Card(huart2); 

    if(strstr((char*)IoT_Protocol_Buf, "+CPIN:"))return SIM_INSERTING; 

    HAL_Delay(2000); 
    timeout--; 
    if(timeout == 0) return SIM_NOT_INSERTED; 
  }
}

void Esp32_WiFi_Init(void)
{
	Esp32_Restart(huart3);
  // HAL_GPIO_WritePin(PhysRestart_Esp32_GPIO_Port, PhysRestart_Esp32_Pin, GPIO_PIN_RESET);
  // HAL_Delay(1500); 
  // HAL_GPIO_WritePin(PhysRestart_Esp32_GPIO_Port, PhysRestart_Esp32_Pin, GPIO_PIN_SET);	
  // if(Esp32_Response(IoT_Protocol_Buf, (uint8_t*)"ready", 1000)) Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte);  
  // else if(Esp32_Response(IoT_Protocol_Buf, (uint8_t*)"WIFI GOT IP", 1000)) Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 
  // else HAL_NVIC_SystemReset(); 
  
  Esp32_Echo(huart3, DISABLE); 
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
  
  Esp32_WiFiMode(huart3, STA_Mode); 
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
  
  /*printf("AT+CWJAP=\"3G\",\"123456779\"\r\n"); 
  HAL_Delay(500); 
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");*/ 
  
  while(!Stm32_Check_Esp32_Connect_WiFi(5))
  {
    WIRELESS_DISPLAY = SmartConfig_WiFi; 
    HAL_Delay(100); 
    
    Esp32_Start_SmartConfig(huart3, TOUTCH_and_AIRKISS_App); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"smartconfig connected wifi"); 
    Esp32_Stop_SmartConfig(huart3);  
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK");
  }
  WIRELESS_DISPLAY = Connected_WiFi; 
  HAL_Delay(100); 
}

void SimA7670C_4GLTE_Init(void)
{
  SimA7670C_Reset(huart2); 
  //Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"SMS DONE"); 
  HAL_Delay(30000); 

  SimA7670C_Echo(huart2, ECHO_DISABLE);
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 

  if(!Stm32_Check_Sim_Card_Connect_Network_Service(5)) Stm32_Restart(); 
  else Stm32_Clear_Buf(IoT_Protocol_Buf, IoT_Byte); 

  WIRELESS_DISPLAY = Network_Service_Standby; 
  HAL_Delay(100); 
}

void Stm32_Restart(void)
{
  Broker_Publish_Topic((uint8_t*)">> Restarting..."); 
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
  HAL_NVIC_SystemReset();
}

void Stm32_Clear_Buf(uint8_t *clr_buff, uint16_t size)
{
  if(clr_buff == &IoT_Protocol_Buf[0]) IoT_Protocol_Count = 0; 
  if(clr_buff == &Inverter_Buf[0]) Inverter_Count = 0; 
	
	for(uint16_t i = 0; i < size; i++) clr_buff[i] = '\0'; 
} 

void Checking_Wireless_Response(uint8_t *recv_buff, uint8_t *content)
{
  if(wirelessProtocol == Esp32_Modules)
  {
    if(!Esp32_Response(recv_buff, content, 10000)) HAL_NVIC_SystemReset(); 
	  else Stm32_Clear_Buf(recv_buff, IoT_Byte);
  }
	else if(wirelessProtocol == SimA7670C_Modules)
  {
    if(!SimA7670C_Response(recv_buff, content, 10000)) HAL_NVIC_SystemReset(); 
    else Stm32_Clear_Buf(recv_buff, IoT_Byte);
  }
}

void Broker_Connect_Init(Server_OptionTypeDef server_name, Module_PeripheralsTypeDef modules)
{
  if(Eeprom_Read_Data(Addr_BrokerFlag, Position_1) != 0xFFFFFFFF)
  {
    //...Read ID from Eeprom 
    sprintf((char*)broker.id, "%ld", Eeprom_Read_Data(Addr_ID, Position_1)); 

    //...Read Username from Eeprom 
    Eeprom_ReadStr_Hexadecimal(Addr_Username, Position_1, Eeprom_Username, Eeprom_Username_Byte, broker.user, Username_Byte); 
    Stm32_Clear_Buf((uint8_t*)Eeprom_Username, Eeprom_Username_Byte); 

    //...Read Password from Eeprom 
    Eeprom_ReadStr_Hexadecimal(Addr_Password, Position_1, Eeprom_Password, Eeprom_Password_Byte, broker.password, Password_Byte); 
    Stm32_Clear_Buf((uint8_t*)Eeprom_Password, Eeprom_Password_Byte);

    //...Read Host from Eeprom 
    Eeprom_ReadStr_Hexadecimal(Addr_Host, Position_1, Eeprom_Host, Eeprom_Host_Byte, broker.host, Host_Byte); 
    Stm32_Clear_Buf((uint8_t*)Eeprom_Host, Eeprom_Host_Byte);

    //...Read Port from Eeprom 
    broker.port = (uint16_t)Eeprom_Read_Data(Addr_Port, Position_2);

    //...Read Topic publish from Eeprom 
    Eeprom_ReadStr_Hexadecimal(Addr_TopicPub, Position_1, Eeprom_TopicPub, Eeprom_TopicPub_Byte, broker.topic.pub, TopicPub_Byte); 
    Stm32_Clear_Buf((uint8_t*)Eeprom_TopicPub, Eeprom_TopicPub_Byte);

    //...Read Topic subscribe from Eeprom 
    Eeprom_ReadStr_Hexadecimal(Addr_TopicSub, Position_1, Eeprom_TopicSub, Eeprom_TopicSub_Byte, broker.topic.sub, TopicSub_Byte); 
    Stm32_Clear_Buf((uint8_t*)Eeprom_TopicSub, Eeprom_TopicSub_Byte);
  }
  else 
  {
    switch(server_name)
    {
      case PRODUCT_SERVER: 
        strcpy((char*)broker.host, "iot-solar.nichietsuvn.com");
        broker.port = 1884; 
        strcpy((char*)broker.user, "guest"); 
        strcpy((char*)broker.password, "123456a@"); 
        strcpy((char*)broker.id, DeviceID); 
        strcpy((char*)broker.topic.sub, "device/"DeviceID"/cmd"); 
        strcpy((char*)broker.topic.pub, "server/"DeviceID"/data"); 
        break; 
      
      case DEVELOPMENT_SERVER:
        strcpy((char*)broker.host, "api-kobell.dev.ncs.int"); 
        broker.port = 1883; 
        strcpy((char*)broker.user, "kobell"); 
        strcpy((char*)broker.password, "12345678@Ab"); 
        strcpy((char*)broker.id, DeviceID); 
        strcpy((char*)broker.topic.sub, "device/"DeviceID"/cmd"); 
        strcpy((char*)broker.topic.pub, "server/"DeviceID"/data"); 
        break; 
      
      case HYTEK_SERVER:
        strcpy((char*)broker.host, "103.7.43.172"); 
        broker.port = 1883;
        strcpy((char*)broker.user, "sensorIsGood"); 
        strcpy((char*)broker.password, "publicIsGood"); 
        strcpy((char*)broker.id, DeviceID); 
        strcpy((char*)broker.topic.sub, "device/"DeviceID"/cmd"); 
        strcpy((char*)broker.topic.pub, "server/"DeviceID"/data"); 
        break; 
    }
  }
  
  if(modules == Esp32_Modules)
  {
    //...Makesure disconnect to Broker 
    Esp32_Mqtt_Disconnect_Broker(huart3); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"ERROR"); 

    Esp32_Mqtt_UserConfig(huart3, broker.id, broker.user, broker.password); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
    HAL_Delay(100);

    Esp32_Mqtt_Connect_Broker(huart3, broker.host, broker.port); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
    HAL_Delay(500);
  }
  else if(modules == SimA7670C_Modules)
  {
    SimA7670C_PDP_Context_Config(huart2, 1, (uint8_t*)IP_PDPTYPE);  
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
    
    SimA7670C_PDP_Context_Set(huart2, 1, PDP_CONTEXT_ACTIVATE); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 

    SimA7670C_MQTT_BrokerDisconnect(huart2); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"ERROR"); 
    
    SimA7670C_MQTT_StopService(huart2); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"ERROR"); 
    
    SimA7670C_MQTT_StartService(huart2); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"+CMQTTSTART:"); 

    SimA7670C_MQTT_Client_Config(huart2, broker.id); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 

    SimA7670C_MQTT_BrokerConnect(huart2, broker.host, broker.port, broker.user, broker.password); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"+CMQTTCONNECT:"); 
  }

  Broker_Publish_Topic((uint8_t*)"<!> Successfully connected"); 
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 

  HAL_Delay(500); 

  Broker_Subscribe_Topic(); 
  Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"OK"); 
}

void Broker_Publish_Topic(uint8_t *msg)
{
  if(wirelessProtocol == Esp32_Modules) Esp32_Mqtt_Publish(huart3, IoT_Protocol_Buf, broker.topic.pub, msg);
  else if(wirelessProtocol == SimA7670C_Modules) 
  {
    Is_Insert_Sim_Card(huart2); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"+CPIN:"); 

    SimA7670C_Mqtt_Publish(huart2, IoT_Protocol_Buf, broker.topic.pub, msg);
  }
}

void Broker_Subscribe_Topic(void)
{
  if(wirelessProtocol == Esp32_Modules) Esp32_Mqtt_Subscribe(huart3, broker.topic.sub);
  else if(wirelessProtocol == SimA7670C_Modules) 
  {
    Is_Insert_Sim_Card(huart2); 
    Checking_Wireless_Response(IoT_Protocol_Buf, (uint8_t*)"+CPIN:"); 

    SimA7670C_Mqtt_Subscribe(huart2, IoT_Protocol_Buf, broker.topic.sub); 
  }
}

unsigned char CheckInput (void)
{
  if(HAL_GPIO_ReadPin(Input_Device_GPIO_Port, Input_Device_Pin) == GPIO_PIN_SET) return 1;
  else return 0;
}

void sendCBSC (void)
{
  if(CheckInput()) Broker_Publish_Topic((uint8_t*)"INPUT-1");
  else if(CheckInput() == 0) Broker_Publish_Topic((uint8_t*)"INPUT-0");
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
