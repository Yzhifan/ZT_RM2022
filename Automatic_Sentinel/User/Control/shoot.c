#include "shoot.h"
#include "drv_can.h"
#include "PID.h"
#include "DR16.h"

/**
* @brief  ·¢Éä¿ØÖÆº¯Êý
* @param  None
* @return None
*/
void shoot_control(void)
{
	int16_t load_out;
	
	if(RC.f>2000)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1360);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1360);
		
		load_out = Increment_PID(&chas4_error,Moto_4.speed,RC.f,8,0.5,0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,400);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,400);
		load_out = Increment_PID(&chas4_error,Moto_4.speed,0,8,0.5,0);
	}
	
	Can_SendMoto_chasiss(load_out,MOTO_ID_4);
}
