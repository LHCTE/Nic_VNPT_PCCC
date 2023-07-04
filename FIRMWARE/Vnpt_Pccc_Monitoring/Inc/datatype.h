#ifndef _DATATYPE_H_
#define _DATATYPE_H_

#include "main.h"

void TextToHexStr(uint8_t *input, uint8_t *output); 
void HexStrToHexChar(uint8_t *input, uint8_t *output); 
void HexCharToHexStr(uint8_t *input, uint16_t size, uint8_t *output); 
void FloatToStr(float value, uint8_t *output); 
float map(float x, float in_min, float in_max, float out_min, float out_max); 
#endif /* _DATATYPE_H_ */ 
