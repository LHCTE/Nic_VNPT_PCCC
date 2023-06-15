#include "relay.h"

Relay_StateTypedef Relay_Control(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, Level_StateTypedef Trigger, Relay_StateTypedef State)
{
	if(State == ON) 
	{
		if(Trigger == HIGH) HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); 
		return ON; 
	}
	else 
	{
		if(Trigger == HIGH) HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); 
		return OFF; 
	}
}
