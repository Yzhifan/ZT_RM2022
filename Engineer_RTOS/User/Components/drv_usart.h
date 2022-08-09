#ifndef __DRV_USART_H
#define __DRV_USART_H

#include "main.h"
#include "stm32f4xx_hal.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;

extern uint8_t controller_data[36];
extern uint8_t Gyro1_Data[22],Gyro2_Data[22];
extern uint16_t RC_Communication;
extern float gyro[3],angle[3],temperate;	

void RC_Init(UART_HandleTypeDef *huart);
void IDLE_Handler(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma_usart_rx);
	
void Gyro_Init(UART_HandleTypeDef *huart,uint8_t *pData);
void IDLE_Gyro_Handler(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma_usart_rx,uint8_t *pData,uint8_t Gyro_number);
	
	
#endif
