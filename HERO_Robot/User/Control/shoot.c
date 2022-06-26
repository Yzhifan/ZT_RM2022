#include "shoot.h"
#include "drv_can.h"
#include "pid.h"
#include "DR16.h"

Error_increment chas5_error;
Error_increment chas6_error;
Error_increment chas7_error;

/**
* @brief  发射电机控制函数
* @param  None
* @return None
*/
void shoot_control(void)
{
		int16_t Lshoot_out,Rshoot_out,Bshoot_out;//左电机、右电机、拨弹电机

//	if(Mouse.L&&Mouse.R==0)
	if(RC.f > 300)
	{
		Lshoot_out = Increment_PID(&chas6_error,Moto_6.speed,-8700,22,1.8,0);
		Rshoot_out = Increment_PID(&chas7_error,Moto_7.speed,8700,22,1.8,0);
		

		if(Moto_6.speed<-8400&&Moto_7.speed>8400)
		{
			Bshoot_out = Increment_PID(&chas5_error,Moto_5.speed,-400,12,1.2,0);
		}
		else Bshoot_out = Increment_PID(&chas5_error,Moto_5.speed,0,12,1.2,0);

//		Can_SendMoto_shoot(-16000,MOTO_ID_6);
//		Can_SendMoto_shoot(16000,MOTO_ID_7);

	}
	
//	else if(Mouse.R == 1)
	else if(RC.f < -300)
	{		
		Lshoot_out = Increment_PID(&chas6_error,Moto_6.speed,0,12,1.2,0);
		Rshoot_out = Increment_PID(&chas7_error,Moto_7.speed,0,12,1.2,0);
		Bshoot_out = Increment_PID(&chas5_error,Moto_5.speed,500,12,1.2,0);

	}
	else {
		Lshoot_out = Increment_PID(&chas6_error,Moto_6.speed,0,12,1.2,0);
		Rshoot_out = Increment_PID(&chas7_error,Moto_7.speed,0,12,1.2,0);
		Bshoot_out = Increment_PID(&chas5_error,Moto_5.speed,0,12,1.2,0);
		
	}
		Can_SendMoto_shoot(Lshoot_out,MOTO_ID_6);
		Can_SendMoto_shoot(Rshoot_out,MOTO_ID_7);
		Can_SendMoto(Bshoot_out,MOTO_ID_5);
	
		
}
