#include "JY901S.h"

int16_t test_angle,test_gyro;
float accel[3],gyro[3],angle[3],temperate;//x��y��z
uint8_t Gyro_Data[44];

void JY901_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);//���UART�����жϱ�־λ
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//ʹ�ܿ����ж�IDLE
  HAL_UART_Receive_DMA(&huart1,Gyro_Data,50);
}

void Gyro_handler(void)
{
	if(Gyro_Data[0]== 0x55)
	{
		/* ���ٶ����ݴ���*/
		if(Gyro_Data[1]== 0x52)
		{
			gyro[0]=((int16_t)(Gyro_Data[2] | (int16_t)Gyro_Data[3] << 8))/32768.0*2000;
			gyro[1]=((int16_t)(Gyro_Data[4] | (int16_t)Gyro_Data[5] << 8))/32768.0*2000;
			gyro[2]=((int16_t)(Gyro_Data[6] | (int16_t)Gyro_Data[7] << 8))/32768.0*2000;
			temperate = ((int16_t)(Gyro_Data[8] | (int16_t)Gyro_Data[9] << 8))/100;
		}
		/* ��̬�����ݴ���*/
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
	
	uint32_t Data_lave,Data_exist; //ʣ��ռ䣬�ѽ��յ��ֽ���
		if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!= RESET))  //�ж��Ƿ��������ж�
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);//���UART�����жϱ�־λ
			
			HAL_UART_DMAStop(&huart1); //�ر�DMA�����ܵ�����
			
			Data_lave = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //��ȡ��������ж�ʱ��DMA��ʣ�Ŀռ��ֽ������ܵĿռ���36���ֽ�

			Data_exist = 44-Data_lave;  //�ܵĿռ��ֽ��� - ʣ��Ŀռ��ֽ��� = �Ѿ����յ��ֽ��� 

			if(Data_exist == 22)  // �������DMA������յ�18���ֽڵ����ݣ������������ȷ�ģ��������ݴ�����
			{
				Gyro_handler();//ң������ƴ�Ӵ���
			}
			HAL_UART_Receive_DMA(&huart1,Gyro_Data,44); //���´�DMA�Ľ��չ���
		}
}

