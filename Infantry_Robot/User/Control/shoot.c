#include "shoot.h"
#include "drv_can.h"
#include "pid.h"
#include "DR16.h"

float p,i,d;
Error_increment chas7_error;

/**
* @brief  发射电机控制函数
* @param  None
* @return None
*/
void shoot_control(void)
{

	int16_t load_out;
	
	if(Mouse.L&&Mouse.R ==0)
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,1360);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,1360);
		
		load_out = Increment_PID(&chas7_error,Moto_7.speed,2000,5,0.15,0);
	}
	else if(Mouse.R)
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,400);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,400);
		load_out = Increment_PID(&chas7_error,Moto_7.speed,-800,5,0.15,0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,400);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,400);
		load_out = Increment_PID(&chas7_error,Moto_7.speed,0,5,0.15,0);
	}
	
	Can_SendMoto_Gimbals(load_out,MOTO_ID_7);
}
