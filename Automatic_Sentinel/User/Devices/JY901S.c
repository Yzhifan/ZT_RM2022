#include "JY901S.h"

int16_t test_angle,test_gyro;
float accel[3],gyro[3],angle[3],temperate;//x、y、z
uint8_t Gyro_Data[44];

void JY901_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除UART空闲中断标志位
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//使能空闲中断IDLE
  HAL_UART_Receive_DMA(&huart1,Gyro_Data,50);
}

void Gyro_handler(void)
{
	if(Gyro_Data[0]== 0x55)
	{
		/* 角速度数据处理*/
		if(Gyro_Data[1]== 0x52)
		{
			gyro[0]=((int16_t)(Gyro_Data[2] | (int16_t)Gyro_Data[3] << 8))/32768.0*2000;
			gyro[1]=((int16_t)(Gyro_Data[4] | (int16_t)Gyro_Data[5] << 8))/32768.0*2000;
			gyro[2]=((int16_t)(Gyro_Data[6] | (int16_t)Gyro_Data[7] << 8))/32768.0*2000;
			temperate = ((int16_t)(Gyro_Data[8] | (int16_t)Gyro_Data[9] << 8))/100;
		}
		/* 姿态角数据处理*/
		if(Gyro_Data[12]== 0x53)
		{
			angle[0]=((int16_t)(Gyro_Data[13] | (int16_t)Gyro_Data[14] << 8))/32768.0*180;
			angle[1]=((int16_t)(Gyro_Data[15] | (int16_t)Gyro_Data[16] << 8))/32768.0*180;
			angle[2]=((int16_t)(Gyro_Data[17] | (int16_t)Gyro_Data[18] << 8))/32768.0*180;
			
			test_angle = angle[2]*22.756f;
		}

	}
}

void IDLE_Gyro_Handler(void)
{
	
	uint32_t Data_lave,Data_exist; //剩余空间，已接收的字节数
		if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET))  //判断是否进入空闲中断
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除UART空闲中断标志位
			
			HAL_UART_DMAStop(&huart1); //关闭DMA避免受到干扰
			
			Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //获取进入空闲中断时，DMA所剩的空间字节数，总的空间是36个字节

			Data_exist = 44-Data_lave;  //总的空间字节数 - 剩余的空间字节数 = 已经接收的字节数 

			if(Data_exist == 22)  // 如果本次DMA传输接收到18个字节的数据，则该数据是正确的，进入数据处理函数
			{
				Gyro_handler();//遥控数据拼接处理
			}
			HAL_UART_Receive_DMA(&huart1,Gyro_Data,44); //重新打开DMA的接收功能
		}
}

