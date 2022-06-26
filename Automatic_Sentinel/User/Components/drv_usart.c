#include "drv_usart.h"


#define ENEMY  1

int16_t test;

uint8_t PC_data[5];
uint8_t uData,number = 0;
int16_t Raw_xData,Raw_yData;
int16_t coordinates_x,coordinates_y;//敌方单位坐标
uint16_t recognize;//识别标志位
float a = 0.8f;//低通滤波系数
int16_t filtered_x,filtered_y;

/**
* @brief  PC端数据处理函数
* @param  None
* @return None
*/
void PC_Handler(void)
{
	
	
	if(PC_data[0] ==81&&PC_data[4] == 82)
	{
		/*上位机识别到目标，置位标志位，改变哨兵工作模式*/
		recognize = PC_data[3];
		
		/*接收并初步转换原始数据*/
		Raw_xData = ((int16_t)(PC_data[1])-40)*22.755f;
		Raw_yData = ((int16_t)(PC_data[2])-40)*22.755f;
		
		/*采用一阶低通滤波对数据进行过滤，增加数据的平稳性，抵消丢帧的影响*/
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
		/*判断帧头是否正确，如果是，则开始对数组赋值*/
		if(uData == 81)
		{
			PC_data[0] = uData;
			number = 1;
			
		}
		/*判断是否为帧尾，如果是，则结束赋值，并进行数据处理*/
		else if(uData == 82)
		{
			test++;
			PC_data[number] = uData;
			number = 0;
			PC_Handler();
		}
		/*非帧头或帧尾，则按照中间数据段赋值，依据number的值确定是数据位的第几个字节*/
		else {
			
			PC_data[number] = uData;
			number++;
		}
		
	}
}


