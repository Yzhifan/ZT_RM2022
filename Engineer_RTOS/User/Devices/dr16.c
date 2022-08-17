#include "dr16.h"


//**����ң�������ݽṹ��**//
 RC_control     RC;   //ң��������ҡ�˵�����							
 Mouse_control  Mouse;// �����ٶ�����
 Mouse_shift    Mouse_move;
 Key_control    Key;  // ���м��̰�ť������

#define RC_sens_X 3     //�����ȣ�ң������ҡ��ֵ����ת����һ��Ϊ ��660��������ڸ���ֵ�и��߱��ʵ�Ҫ�󣬿���ͨ���˴��Ŵ�
#define RC_sens_Y 1
#define RC_sens_Z 3
#define RC_sens_pitch 1
#define RC_sens_F  1
#define Mouse_sens  2

#define Gyro_mode 1  //���������ǻش������ݿ�����̨
#define Motor_mode 3 //���ݵ������Ļ�е�Ƕȿ�����̨

#define Verify_DR16  0x800

int16_t Mouse_move_gyro_y;

extern EventGroupHandle_t VerifyHandle;

/**
* @brief  ң�����ݴ�����
* @param  None
* @return None
*/
void Controller_handler(void)
{
	xEventGroupSetBitsFromISR(VerifyHandle,Verify_DR16,0);
	
  unsigned short ch[5];
	
	//*ң����ҡ������*//
	ch[0] = ((int16_t)controller_data[0] | ((int16_t)controller_data[1] << 8)) & 0x07FF;
	ch[1] = (((int16_t)controller_data[1] >> 3) | ((int16_t)controller_data[2] << 5)) & 0x07FF;
	ch[2] = (((int16_t)controller_data[2] >> 6) | ((int16_t)controller_data[3] << 2) | ((int16_t)controller_data[4] << 10)) & 0x07FF;
	ch[3] = (((int16_t)controller_data[4] >> 1) | ((int16_t)controller_data[5]<<7)) & 0x07FF;
	ch[4] = ((int16_t)controller_data[16]|(int16_t)controller_data[17] << 8) & 0x07FF;
	RC.x     = ( ch[2]-1024 ) * RC_sens_X;
	RC.y     = ( ch[3]-1024 ) * RC_sens_Y;
	RC.z     = ( ch[0]-1024 ) * RC_sens_Z;
	RC.pitch = ( ch[1]-1024 ) * RC_sens_pitch;
	RC.f     = ( ch[4]-1024 ) * RC_sens_F;
	
	//*ң����ͨ����������*//
	RC.s1 = ((controller_data[5] >> 4) & 0x000C) >> 2;
	RC.s2 = ((controller_data[5] >> 4) & 0x0003);
	
	//*���̰�������*//
	Key.Board =  ((int16_t)controller_data[14]) | ((int16_t)controller_data[15]<<8);
	Key.W = ((Key.Board>>0)&0x01);
	Key.S = ((Key.Board>>1)&0x01);
	Key.A = ((Key.Board>>2)&0x01);
	Key.D = ((Key.Board>>3)&0x01);
	Key.SHIFT = ((Key.Board>>4)&0x01);
	Key.CTRL  = ((Key.Board>>5)&0x01);
	Key.Q = ((Key.Board>>6)&0x01);
	Key.E = ((Key.Board>>7)&0x01);
	Key.R = ((Key.Board>>8)&0x01);
	Key.F = ((Key.Board>>9)&0x01);
	Key.G = ((Key.Board>>10)&0x01);
	Key.Z = ((Key.Board>>11)&0x01);
	Key.X = ((Key.Board>>12)&0x01);
	Key.C = ((Key.Board>>13)&0x01);
	Key.V = ((Key.Board>>14)&0x01);
	Key.B = ((Key.Board>>15)&0x01);
	
	
	//*����ٶ�����*//
	Mouse.x = (((int16_t)controller_data[6]) | ((int16_t)controller_data[7] << 8))*Mouse_sens;
	Mouse.y = -(((int16_t)controller_data[8]) | ((int16_t)controller_data[9] << 8))*Mouse_sens;
	Mouse.z = ((int16_t)controller_data[10]) | ((int16_t)controller_data[11] << 8);
	Mouse.L = controller_data[12];
	Mouse.R = controller_data[13];
	
	//*���λ������*//
	if(Key.SHIFT == 0)
	{
		Mouse_move.x += (float)Mouse.x;
		if(Mouse_move.x >4095)
			Mouse_move.x = -4095;
		else if(Mouse_move.x < -4095)
		{
			Mouse_move.x =4095;
		}
		

		if(Mouse_move.y<600.0f&&Mouse_move.y>-250.0f)
		{
			Mouse_move.y += (float)Mouse.y;
		}
		else if(Mouse_move.y>=600.0f)
		{
			if(Mouse.y<0)
			{
				Mouse_move.y += (float)Mouse.y;
			}
		}
		else if(Mouse_move.y<=-250.0f)
		{
			if(Mouse.y>0)
			{
				Mouse_move.y += (float)Mouse.y;
			}
		}
	}
  
//	}
//	/*������ģʽ�µ���̬�Ƕ���λ*/
//	else if(RC.s1 == Gyro_mode)
//	{
//		if(Mouse_move_gyro_y<160.0f&&Mouse_move_gyro_y>-240.0f)
//		{
//			Mouse_move_gyro_y += (float)Mouse.y;
//		}
//		else if(Mouse_move_gyro_y>=160.0f)
//		{
//			if(Mouse.y<0)
//			{
//				Mouse_move_gyro_y += (float)Mouse.y;
//			}
//		}
//		else if(Mouse_move_gyro_y<=-240.0f)
//		{
//			if(Mouse.y>0)
//			{
//				Mouse_move_gyro_y += (float)Mouse.y;
//			}
//		}
//	}


		
}
