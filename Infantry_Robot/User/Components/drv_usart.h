#ifndef __DRV_USART_H
#define __DRV_USART_H

#include "main.h"
#include "stm32f4xx_hal.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern uint8_t controller_data[36];
extern uint16_t RC_Communication;
extern float gyro[3],angle[3],temperate;	

void RC_Init(void);
void IDLE_Handler(void);
	
void JY901_Init(void);
void IDLE_Gyro_Handler(void);
	
	
#endif
