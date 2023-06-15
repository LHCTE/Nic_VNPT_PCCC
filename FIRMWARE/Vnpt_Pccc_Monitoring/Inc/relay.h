#ifndef _RELAY_CONTROL_H_
#define _RELAY_CONTROL_H_

#include "main.h"

typedef enum { OFF = 0, ON = !OFF } Relay_StateTypedef; 
typedef enum { LOW = 0, HIGH = !LOW } Level_StateTypedef; 

Relay_StateTypedef Relay_Control(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, Level_StateTypedef Trigger, Relay_StateTypedef State); 

#endif /* _RELAY_CONTROL_H_ */ 
