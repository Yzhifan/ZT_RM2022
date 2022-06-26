#ifndef __JY901S_H
#define __JY901S_H

#include "main.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern float accel[3],gyro[3],angle[3],temperate;

void JY901_Init(void);
void IDLE_Gyro_Handler(void);


#endif
