#include "drv_usart.h"
#include "dr16.h"
#include "jy901s.h"


#define Signal_normal 1     //遥控信号通讯检验标志，当通讯正常时，标志位赋 1 否则在中断中持续增加，直至达到一定大小，让机器人停止


uint8_t  controller_data[36];//定义一个缓冲数组，用来接收18帧的数据，同时留下一部分空间防止数据过冲
uint16_t RC_Communication;

uint8_t Gyro_Data[22];


/**
* @brief  遥控器接收初始化函数
* @param  None
* @return None
*/
void RC_Init(void)
{
	
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除UART空闲中断标志位
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);//使能空闲中断IDLE
  HAL_UART_Receive_DMA(&huart3,controller_data,36);

}


/**
* @brief  遥控器DMA空闲中断校验数据函数
* @param  None
* @return None
*/
void IDLE_Handler(void)
{
	uint32_t Data_lave,Data_exist; //剩余空间，已接收的字节数
		if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!= RESET))  //判断是否进入空闲中断
		{
			
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除UART空闲中断标志位
			
			HAL_UART_DMAStop(&huart3); //关闭DMA避免受到干扰
			
			Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx); //获取进入空闲中断时，DMA所剩的空间字节数，总的空间是36个字节

			Data_exist = 36-Data_lave;  //总的空间字节数 - 剩余的空间字节数 = 已经接收的字节数 

			if(Data_exist == 18)  // 如果本次DMA传输接收到18个字节的数据，则该数据是正确的，进入数据处理函数
			{
				RC_Communication = Signal_normal;
				Controller_handler();//遥控数据拼接处理
			}
			HAL_UART_Receive_DMA(&huart3,controller_data,36); //重新打开DMA的接收功能
		}
}

/**
* @brief  陀螺仪接收初始化函数
* @param  None
* @return None
*/
void JY901_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1,Gyro_Data,22);
}
/**
* @brief  陀螺仪DMA空闲中断校验数据函数
* @param  None
* @return None
*/
void IDLE_Gyro_Handler(void)
{
	
	uint32_t Data_lave,Data_exist; 
		if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET))  
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			
			HAL_UART_DMAStop(&huart1); 
			
			Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); 

			Data_exist = 22-Data_lave; 

			if(Data_exist == 11)  
			{
				Gyro_handler();
			}
			HAL_UART_Receive_DMA(&huart1,Gyro_Data,22); 
		}
}
