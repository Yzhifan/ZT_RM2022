#ifndef __OSCILLOSCOPE_H
#define __OSCILLOSCOPE_H

#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart6;
extern float OutData[4];

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OutPut_Data(void);


#endif
