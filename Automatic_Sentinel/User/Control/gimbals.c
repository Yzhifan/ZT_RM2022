#include "gimbals.h"
#include "PID.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"

#define Left  0 //向左巡航
#define Right 1 //向右巡航

#define Drop  0 //下降
#define Rise  1 //上升

#define Cruise 0 //巡航
#define Track  1 //跟踪


extern float in_p,in_i,in_d,out_p,out_i,out_d;

int8_t tims;
int16_t yaw_exp,pitch_exp;
uint8_t turn_sign,lift_sign;//转向标志位、抬升标志位
uint8_t Gimbals_Mode;//云台模式标志(巡航模式、跟踪模式切换)
int16_t yaw_position_out,pitch_position_out;


/**
* @brief  云台巡航函数，通过累加值让云台以特定的速度巡视周围
* @param  None
* @return None
*/
void gimbals_cruise(void)
{
	/*判断上下俯仰标志位的状态*/
	if(lift_sign == Rise)
	{
		/*判断pitch轴角度是否到达限位，是则改变俯仰标志位的转向*/
		if(pitch_exp >= 300 )
		{
			lift_sign = Drop;
		}
		/*否则对pitch轴期望角度值进行累加*/
		else pitch_exp+=2;
	}
	else if(lift_sign == Drop)
	{
		
		if(pitch_exp <= -300)
		{
			lift_sign = Rise;
		}
		else pitch_exp-=2;
	}
	/*判断左右转向标志位的状态*/
	if(turn_sign == Right)
	{
		/*判断yaw轴角度是否到达限位，是则改变转向标志位的转向*/
		if(yaw_exp >=-1600)
		{
			turn_sign = Left;
		}
		else yaw_exp+=6;
	}
	else if(turn_sign == Left)
	{
		
		if(yaw_exp<=-2800)
		{
			turn_sign = Right;
		}
		else yaw_exp-=6;
	}
}


/**
* @brief  云台控制函数
* @param  None
* @return None
*/
void gimbals_control(void)
{
	int16_t yaw_out,pitch_out;
	
	/*跟踪状态和循环状态判断*/
 switch(recognize)
 {	
	 /*识别标志位为0，将云台切换到巡航模式*/
	 case 0:
	 {
		 Gimbals_Mode = Cruise;
		 

	 }break;
	 /*检测识别标志位是否为1，是则将云台模式转换为跟踪模式*/
	 case 1:
	 {
		 Gimbals_Mode = Track;
		/*使云台从结束跟踪模式后的机械角度开始巡航*/
		 pitch_exp = pitch_angle;
		 yaw_exp = yaw_angle;
	 }break;
	

 }
 
 /*判断云台所处模式，根据云台的模式选择对应的pid函数运行*/
	if(Gimbals_Mode == Cruise)
	{
		gimbals_cruise();
		yaw_out   = Gimbals_loops_pid(&yaw_angle_error,&yaw_gyro_error,yaw_exp,yaw_angle,yaw_speed);
		pitch_out = Gimbals_loops_pid(&pitch_angle_error,&pitch_gyro_error,pitch_exp,pitch_angle,pitch_speed);
	}
	 else if(Gimbals_Mode == Track)
	{

		yaw_out = yaw_loops_pid(&yaw_track_error,&yaw_gyro_error,coordinates_x,yaw_speed);
		pitch_out = pitch_loops_pid(&pitch_track_error,&pitch_gyro_error,coordinates_y,pitch_speed);
		
	}
	else
	{
		yaw_out = 0;
		pitch_out = 0;
	}

	Can_SendMoto_gimbals(yaw_out,MOTO_ID_5);
	Can_SendMoto_gimbals(pitch_out,MOTO_ID_6);
	
}



