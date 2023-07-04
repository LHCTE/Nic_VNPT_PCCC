#include "datatype.h"
#include <stdio.h> 
#include <string.h>

void TextToHexStr(uint8_t *input, uint8_t *output)
{
	uint16_t i = 0;
	uint16_t loop = 0;
	
	while(input[loop] != '\0')
	{
		sprintf((char*)(output + i),"%02X", input[loop]);
		loop += 1;
		i += 2;
	}
	
	//insert NULL at the end of the output string
	output[i++] = '\0';
}

void HexStrToHexChar(uint8_t *input, uint8_t *output)
{
	for(uint8_t i = 0; i < strlen((char*)input) / 2; i++) 
	{ 
		if(input[i * 2] >= 48 && input[i * 2] <= 57) output[i] = (input[i * 2] - 48) << 4; //...Number HEX      	
		else if(input[i * 2] >= 97 && input[i * 2] <= 102) output[i] = (input[i * 2] - 32) << 4; //...Lowercase Character HEX
		else if(input[i * 2] >= 65 && input[i * 2] <= 70) output[i] = (input[i * 2] - 55) << 4; //...Uppercase  Character HEX
			
			
		if(input[i * 2 + 1] >= 48 && input[i * 2 + 1] <= 57) output[i] |= input[i * 2 + 1] - 48; //...Number HEX
		else if(input[i * 2 + 1] >= 97 && input[i * 2 + 1] <= 102) output[i] |= input[i * 2 + 1] - 32; //...Lowercase Character HEX
		else if(input[i * 2 + 1] >= 65 && input[i * 2 + 1] <= 70) output[i] |= input[i * 2 + 1] - 55; //...Uppercase  Character HEX
  	}
}

void HexCharToHexStr(uint8_t *input, uint16_t size, uint8_t *output)
{
	for(uint16_t i = 0; i < size; i++) 
	{
		if ((input[i] & 0x0F) < 10) output[(i * 2) + 1] = (input[i] & 0x0F) + 48;
		else output[(i * 2) + 1] = (input[i] & 0x0F) + 55;
		
		if ((input[i] >> 4) < 10) output[i * 2] = (input[i] >> 4) + 48;
		else output[i * 2] = (input[i] >> 4) + 55;
	}
}

void FloatToStr(float value, uint8_t *output) 
{
    int dVal, dec, i;

    value += 0.005;   // added after a comment from Matt McNabb, see below.

    dVal = value;
    dec = (int)(value * 100) % 100;

    memset(output, 0, 100);
    output[0] = (dec / 10) + '0';
    //output[1] = (dec % 10) + '0';
    output[1] = '.';

    i = 2;
    while (dVal > 0)
    {
        output[i] = (dVal % 10) + '0';
        dVal /= 10;
        i++;
    }

    for (i=strlen((char*)output)-1; i>=0; i--)
        putc(output[i], stdout);
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}