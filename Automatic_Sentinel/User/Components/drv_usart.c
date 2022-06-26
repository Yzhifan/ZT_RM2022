#include "drv_usart.h"


#define ENEMY  1

int16_t test;

uint8_t PC_data[5];
uint8_t uData,number = 0;
int16_t Raw_xData,Raw_yData;
int16_t coordinates_x,coordinates_y;//�з���λ����
uint16_t recognize;//ʶ���־λ
float a = 0.8f;//��ͨ�˲�ϵ��
int16_t filtered_x,filtered_y;

/**
* @brief  PC�����ݴ�����
* @param  None
* @return None
*/
void PC_Handler(void)
{
	
	
	if(PC_data[0] ==81&&PC_data[4] == 82)
	{
		/*��λ��ʶ��Ŀ�꣬��λ��־λ���ı��ڱ�����ģʽ*/
		recognize = PC_data[3];
		
		/*���ղ�����ת��ԭʼ����*/
		Raw_xData = ((int16_t)(PC_data[1])-40)*22.755f;
		Raw_yData = ((int16_t)(PC_data[2])-40)*22.755f;
		
		/*����һ�׵�ͨ�˲������ݽ��й��ˣ��������ݵ�ƽ���ԣ�������֡��Ӱ��*/
		coordinates_x = (int16_t)(a * Raw_xData + (1 - a ) * filtered_x);
		coordinates_y = (int16_t)(a * Raw_yData + (1 - a ) * filtered_y);
		
		filtered_x = coordinates_x;
		filtered_y = coordinates_y;
//		HAL_UART_Transmit(&huart1,PC_data,5,10);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)   
{
	
	if(huart ==&huart1)
	{
		/*�ж�֡ͷ�Ƿ���ȷ������ǣ���ʼ�����鸳ֵ*/
		if(uData == 81)
		{
			PC_data[0] = uData;
			number = 1;
			
		}
		/*�ж��Ƿ�Ϊ֡β������ǣ��������ֵ�����������ݴ���*/
		else if(uData == 82)
		{
			test++;
			PC_data[number] = uData;
			number = 0;
			PC_Handler();
		}
		/*��֡ͷ��֡β�������м����ݶθ�ֵ������number��ֵȷ��������λ�ĵڼ����ֽ�*/
		else {
			
			PC_data[number] = uData;
			number++;
		}
		
	}
}


