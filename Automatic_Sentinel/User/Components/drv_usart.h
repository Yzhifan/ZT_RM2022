#ifndef __DRV_USART_H
#define __DRV_USART_H

#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern uint8_t PC_data[5],uData;
extern int16_t Raw_xData,Raw_yData;
extern int16_t coordinates_x,coordinates_y;//�з���λ����
extern uint16_t recognize;//ʶ���־λ
extern float a;

#endif
