#include "drv_usart.h"
#include "jy901s.h"

extern EventGroupHandle_t VerifyHandle;

float gyro[3],arm1_angle[3],arm2_angle[3],temperate;

#define Verify_Gyro_1  0x200
#define Verify_Gyro_2  0x400

/**
* @brief  ���������ݴ�����
* @param  �����Ǽ���
* @return None
*/
void Gyro_handler(uint8_t number)
{
	if(number == 1)
	{
		if(Gyro1_Data[0]== 0x55)
		{
		/*��������200Hz����̬������*/
		arm1_angle[0]=((int16_t)(Gyro1_Data[2] | (int16_t)Gyro1_Data[3] << 8))/32768.0*180;
		arm1_angle[1]=((int16_t)(Gyro1_Data[4] | (int16_t)Gyro1_Data[5] << 8))/32768.0*180;
		arm1_angle[2]=((int16_t)(Gyro1_Data[6] | (int16_t)Gyro1_Data[7] << 8))/32768.0*180;
		
		xEventGroupSetBitsFromISR(VerifyHandle,Verify_Gyro_1,0);	
			
		/*���ո�100Hz�Ľ��ٶ����ݺ���̬������*/
//		gyro[0]=((int16_t)(Gyro1_Data[2] | (int16_t)Gyro1_Data[3] << 8))/32768.0*2000;
//		gyro[1]=((int16_t)(Gyro1_Data[4] | (int16_t)Gyro1_Data[5] << 8))/32768.0*2000;
//		gyro[2]=((int16_t)(Gyro1_Data[6] | (int16_t)Gyro1_Data[7] << 8))/32768.0*2000;
//		temperate = ((int16_t)(Gyro1_Data[8] | (int16_t)Gyro1_Data[9] << 8))/100;

		}
	}
	else if(number == 2)
	{
		if(Gyro2_Data[0]== 0x55)
		{
		/*��������200Hz����̬������*/
		arm2_angle[0]=((int16_t)(Gyro2_Data[2] | (int16_t)Gyro2_Data[3] << 8))/32768.0*180;
		arm2_angle[1]=((int16_t)(Gyro2_Data[4] | (int16_t)Gyro2_Data[5] << 8))/32768.0*180;
		arm2_angle[2]=((int16_t)(Gyro2_Data[6] | (int16_t)Gyro2_Data[7] << 8))/32768.0*180;
			
		xEventGroupSetBitsFromISR(VerifyHandle,Verify_Gyro_2,0);	
		
		/*���ո�100Hz�Ľ��ٶ����ݺ���̬������*/
//		gyro[0]=((int16_t)(Gyro2_Data[2] | (int16_t)Gyro2_Data[3] << 8))/32768.0*2000;
//		gyro[1]=((int16_t)(Gyro2_Data[4] | (int16_t)Gyro2_Data[5] << 8))/32768.0*2000;
//		gyro[2]=((int16_t)(Gyro2_Data[6] | (int16_t)Gyro2_Data[7] << 8))/32768.0*2000;
//		temperate = ((int16_t)(Gyro2_Data[8] | (int16_t)Gyro2_Data[9] << 8))/100;

		
		}
		
	}

	
}



