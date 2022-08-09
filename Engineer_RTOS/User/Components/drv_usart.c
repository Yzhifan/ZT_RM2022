#include "drv_usart.h"
#include "dr16.h"
#include "jy901s.h"


#define Signal_normal 1     //ң���ź�ͨѶ�����־����ͨѶ����ʱ����־λ�� 1 �������ж��г������ӣ�ֱ���ﵽһ����С���û�����ֹͣ


uint8_t  controller_data[36];//����һ���������飬��������18֡�����ݣ�ͬʱ����һ���ֿռ��ֹ���ݹ���
uint16_t RC_Communication;

uint8_t Gyro1_Data[22],Gyro2_Data[22];


/**
* @brief  ң�������ճ�ʼ������
* @param  ����uartͨѶ�ڵĽṹ��ָ��
* @return None
*/
void RC_Init(UART_HandleTypeDef *huart)
{

	__HAL_UART_CLEAR_IDLEFLAG(huart);//���UART�����жϱ�־λ
	__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);//ʹ�ܿ����ж�IDLE
  HAL_UART_Receive_DMA(huart,controller_data,36);

}


/**
* @brief  ң����DMA�����ж�У�����ݺ���
* @param  ����uartͨѶ�ڵĽṹ��ָ��
* @param  ����DMAͨ���Ľṹ��ָ��
* @return None
*/
void IDLE_Handler(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma_usart_rx)
{
	uint32_t Data_lave,Data_exist; //ʣ��ռ䣬�ѽ��յ��ֽ���
		if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)!= RESET))  //�ж��Ƿ��������ж�
		{
			
			__HAL_UART_CLEAR_IDLEFLAG(huart);//���UART�����жϱ�־λ
			
			HAL_UART_DMAStop(huart); //�ر�DMA�����ܵ�����
			
			Data_lave = __HAL_DMA_GET_COUNTER(hdma_usart_rx); //��ȡ��������ж�ʱ��DMA��ʣ�Ŀռ��ֽ������ܵĿռ���36���ֽ�

			Data_exist = 36-Data_lave;  //�ܵĿռ��ֽ��� - ʣ��Ŀռ��ֽ��� = �Ѿ����յ��ֽ��� 

			if(Data_exist == 18)  // �������DMA������յ�18���ֽڵ����ݣ������������ȷ�ģ��������ݴ�����
			{
				RC_Communication = Signal_normal;
				Controller_handler();//ң������ƴ�Ӵ���
			}
			HAL_UART_Receive_DMA(huart,controller_data,36); //���´�DMA�Ľ��չ���
		}
}

/**
* @brief  �����ǽ��ճ�ʼ������
* @param  ����uartͨѶ�ڵĽṹ��ָ��
* @return None
*/
void Gyro_Init(UART_HandleTypeDef *huart,uint8_t *pData)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
  HAL_UART_Receive_DMA(huart,pData,22);
}
/**
* @brief  ������DMA�����ж�У�����ݺ���
* @param  ����uartͨѶ�ڵĽṹ��ָ��
* @param  ����DMAͨ���Ľṹ��ָ��
* @param  �����ǵı��
* @return None
*/
void IDLE_Gyro_Handler(UART_HandleTypeDef *huart,DMA_HandleTypeDef *hdma_usart_rx,uint8_t *pData,uint8_t Gyro_number)
{
	
	uint32_t Data_lave,Data_exist; 
		if((__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)!= RESET))  
		{
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			
			HAL_UART_DMAStop(huart); 
			
			Data_lave = __HAL_DMA_GET_COUNTER(hdma_usart_rx); 

			Data_exist = 22-Data_lave; 

			if(Data_exist == 11)  
			{
				Gyro_handler(Gyro_number);
			}
			HAL_UART_Receive_DMA(huart,pData,22); 
		}
}
