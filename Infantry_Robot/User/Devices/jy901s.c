#include "drv_usart.h"
#include "drv_can.h"
#include "jy901s.h"

float gyro[3],angle[3],temperate;
int16_t yaw_gyro_angle,pitch_gyro_angle;

/**
* @brief  陀螺仪数据处理函数
* @param  None
* @return None
*/
void Gyro_handler(void)
{
	if(Gyro_Data[0]== 0x55)
	{
		/*单独接收200Hz的姿态角数据*/
		angle[0]=((int16_t)(Gyro_Data[2] | (int16_t)Gyro_Data[3] << 8))/32768.0*180;
		angle[1]=((int16_t)(Gyro_Data[4] | (int16_t)Gyro_Data[5] << 8))/32768.0*180;
		angle[2]=((int16_t)(Gyro_Data[6] | (int16_t)Gyro_Data[7] << 8))/32768.0*180;
		
		/*接收各100Hz的角速度数据和姿态角数据*/
//		gyro[0]=((int16_t)(Gyro_Data[2] | (int16_t)Gyro_Data[3] << 8))/32768.0*2000;
//		gyro[1]=((int16_t)(Gyro_Data[4] | (int16_t)Gyro_Data[5] << 8))/32768.0*2000;
//		gyro[2]=((int16_t)(Gyro_Data[6] | (int16_t)Gyro_Data[7] << 8))/32768.0*2000;
//		temperate = ((int16_t)(Gyro_Data[8] | (int16_t)Gyro_Data[9] << 8))/100;
//		
//		angle[0]=((int16_t)(Gyro_Data[13] | (int16_t)Gyro_Data[14] << 8))/32768.0*180;
//		angle[1]=((int16_t)(Gyro_Data[15] | (int16_t)Gyro_Data[16] << 8))/32768.0*180;
//		angle[2]=((int16_t)(Gyro_Data[17] | (int16_t)Gyro_Data[18] << 8))/32768.0*180;
		
		/*将接收到的姿态角由浮点型转换为整型数据，并和电机机械角对应*/
		pitch_gyro_angle = angle[1] * 22.755f;
		yaw_gyro_angle   = angle[2] * 22.755f;
		
	}
}



