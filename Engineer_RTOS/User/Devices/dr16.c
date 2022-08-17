#include "dr16.h"


//**声明遥控器数据结构体**//
 RC_control     RC;   //遥控器所有摇杆的数据							
 Mouse_control  Mouse;// 鼠标的速度数据
 Mouse_shift    Mouse_move;
 Key_control    Key;  // 所有键盘按钮的数据

#define RC_sens_X 3     //灵敏度，遥控器的摇杆值经过转换，一般为 ±660，如果对于该数值有更高倍率的要求，可以通过此处放大
#define RC_sens_Y 1
#define RC_sens_Z 3
#define RC_sens_pitch 1
#define RC_sens_F  1
#define Mouse_sens  2

#define Gyro_mode 1  //根据陀螺仪回传的数据控制云台
#define Motor_mode 3 //根据电机自身的机械角度控制云台

#define Verify_DR16  0x800

int16_t Mouse_move_gyro_y;

extern EventGroupHandle_t VerifyHandle;

/**
* @brief  遥控数据处理函数
* @param  None
* @return None
*/
void Controller_handler(void)
{
	xEventGroupSetBitsFromISR(VerifyHandle,Verify_DR16,0);
	
  unsigned short ch[5];
	
	//*遥控器摇杆数据*//
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
	
	//*遥控器通道拨杆数据*//
	RC.s1 = ((controller_data[5] >> 4) & 0x000C) >> 2;
	RC.s2 = ((controller_data[5] >> 4) & 0x0003);
	
	//*键盘按键数据*//
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
	
	
	//*鼠标速度数据*//
	Mouse.x = (((int16_t)controller_data[6]) | ((int16_t)controller_data[7] << 8))*Mouse_sens;
	Mouse.y = -(((int16_t)controller_data[8]) | ((int16_t)controller_data[9] << 8))*Mouse_sens;
	Mouse.z = ((int16_t)controller_data[10]) | ((int16_t)controller_data[11] << 8);
	Mouse.L = controller_data[12];
	Mouse.R = controller_data[13];
	
	//*鼠标位移数据*//
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
//	/*陀螺仪模式下的姿态角度限位*/
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
