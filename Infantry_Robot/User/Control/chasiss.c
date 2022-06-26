#include <math.h>
#include "chasiss.h"
#include "drv_can.h"
#include "pid.h"
#include "dr16.h"
#include "jy901s.h"

#define Mult 3       //底盘速度倍率
#define Forward   10 //正向移动速度
#define Sideways  6  //横向移动速度
#define Rotate    6  //旋转移动速度

#define Follow_mode   1 //跟随模式
#define Joystick_mode 2 //遥控器拨杆模式
#define Liberty_mode  3 //自由模式


int16_t chassis_value[4],chas_speed[3] ;
int16_t chas_x,chas_y,chas_z;
float mechanic_angle;//电机机械角
float angle_error;
int16_t y_exp,x_exp;

/*底盘电机结构体*/
Error_increment chas1_error;
Error_increment chas2_error;
Error_increment chas3_error;
Error_increment chas4_error;

Error_position  chas_follow;
Error_position  chas_angle_error;	
Error_position  chas_gyro_error;

/**
* @brief  底盘运动缓启动函数
* @param  要缓冲的目标，缓冲倍率
* @return 缓冲后的目标值
*/
int16_t tardy_start(int16_t goal,int16_t value)
{
	if(goal < value)
	{
		goal++;
	}
	return goal;
}

/**
* @brief  底盘运动缓停函数
* @param  要缓冲的目标，缓冲数值
* @return 缓冲后的目标值
*/
int16_t tardy_stop(int16_t goal,int16_t value)
{
	if(goal > 0)
	{
		goal-=value;
	}
	else if(goal < 0)
	{
		goal+=value;
	}
	return goal;
}
/**
* @brief  底盘控制运动函数
* @param  None
* @return None
*/
void chasiss_control(void)
{
	int16_t go_exp[4];
	
	
	/*遥控器拨杆模式*/
	if(RC.s2 == Joystick_mode)
	{
		go_exp[0] = Mult * (RC.x + RC.y + RC.z);//将遥控器各个方向的数据结合成一个电机的期望参数
		go_exp[1] = Mult * (RC.x - RC.y + RC.z);
		go_exp[2] = Mult * (-RC.x - RC.y + RC.z);
		go_exp[3] = Mult * (-RC.x + RC.y + RC.z);
		
	}
	/*自由运动模式*/
  else if(RC.s2 == Liberty_mode)
	{
				/*小陀螺模式*/
		if(Key.SHIFT)
		{
			Gyro_Chasiss();
		}
		else 
		{
				/*前进后退*/
			if(Key.W)
			{
				chas_speed[0] = tardy_start(chas_speed[0],200);
				chas_y = chas_speed[0] * Forward;
			}
			else if(Key.S)
			{
				chas_speed[0] = tardy_start(chas_speed[0],200);
				chas_y = - chas_speed[0] * Forward;
			}
			else 
			{
				chas_y = tardy_stop(chas_y,10);
				chas_speed[0] = 0;
			}
			/*左右侧移*/
			if(Key.A)
			{
				chas_speed[1] = tardy_start(chas_speed[1],200);
				chas_x = - chas_speed[1] * Sideways;
			}
			else if(Key.D)
			{
				chas_speed[1] = tardy_start(chas_speed[1],200);
				chas_x = chas_speed[1] * Sideways;
			}
			else 
			{
				chas_x = tardy_stop(chas_x,10);
				chas_speed[1] = 0;
			}
			/*旋转移动*/
			if(Key.Q)
			{
				chas_speed[2] = tardy_start(chas_speed[2],200);
				chas_z = -chas_speed[2] * Rotate;
			}
			else if(Key.E)
			{
				chas_speed[2] = tardy_start(chas_speed[2],200);
				chas_z = chas_speed[2] * Rotate;
			}
			else 
			{
				chas_z = 0;
				chas_speed[2] = 0;
			}
		}
		
		/*各个方向的数据拟合*/
		go_exp[0] = (chas_x + chas_y + chas_z);
		go_exp[1] = (chas_x - chas_y + chas_z);
		go_exp[2] = (-chas_x - chas_y + chas_z);
		go_exp[3] = (-chas_x + chas_y + chas_z);

	}
	/*跟随云台模式*/
	else if(RC.s2 == Follow_mode)
	{
		/*小陀螺模式*/
		if(Key.SHIFT)
		{
			Gyro_Chasiss();
		}
		else
		{
			/*前进后退*/
			if(Key.W)
			{
				chas_speed[0] = tardy_start(chas_speed[0],200);
				chas_y = chas_speed[0] * Forward;
			}
			else if(Key.S)
			{
				chas_speed[0] = tardy_start(chas_speed[0],200);
				chas_y = - chas_speed[0] * Forward;
			}
			else 
			{
				chas_y = tardy_stop(chas_y,20);
				chas_speed[0] = 0;
			}
			/*左右侧移*/
			if(Key.A)
			{
				chas_speed[1] = tardy_start(chas_speed[1],200);
				chas_x = - chas_speed[1] * Sideways;
			}
			else if(Key.D)
			{
				chas_speed[1] = tardy_start(chas_speed[1],200);
				chas_x = chas_speed[1] * Sideways;
			}
			else 
			{
				chas_x = tardy_stop(chas_x,10);
				chas_speed[1] = 0;
			}
			
			/*旋转移动*/
			
	//		if(yaw_angle>50|yaw_angle<-50)
	//		{
	//			chas_z = Position_Aangle_pid(&chas_follow,yaw_angle,0,2.4,0,0.8);
	//		}
	//		else chas_z = 0;

			chas_z = Position_Aangle_pid(&chas_follow,yaw_angle,0,2.4,0,1.2);
		
		}
		/*各个方向的数据拟合*/
		go_exp[0] = ( chas_x + chas_y + chas_z);
		go_exp[1] = ( chas_x - chas_y + chas_z);
		go_exp[2] = (-chas_x - chas_y + chas_z);
		go_exp[3] = (-chas_x + chas_y + chas_z);
		
		
	}

	/*底盘电机PID计算*/
	chassis_value[0] = Increment_PID(&chas1_error,Moto_1.speed,go_exp[0],12,1.2,0);//一号电机pid运算
	chassis_value[1] = Increment_PID(&chas2_error,Moto_2.speed,go_exp[1],12,1.2,0);
	chassis_value[2] = Increment_PID(&chas3_error,Moto_3.speed,go_exp[2],12,1.2,0);
	chassis_value[3] = Increment_PID(&chas4_error,Moto_4.speed,go_exp[3],12,1.2,0);
	
	/*判断底盘电机速度是否正常，如若不正常则对电机进行断电处理*/
	if((Moto_1.speed&&Moto_2.speed&&Moto_3.speed&&Moto_4.speed) <= 4000)
	{
		Can_SendMoto_Chassis(chassis_value[0],chassis_value[1],chassis_value[2],chassis_value[3]);
	}
	else Can_SendMoto_Chassis(0,0,0,0);
}

void Gyro_Chasiss(void)
{
	mechanic_angle = yaw_angle/22.755f;//电机机械角8192格转换成360°
	angle_error = (angle[2]-mechanic_angle)*0.0174533f; //角度误差 = （陀螺仪角度 - 机械角）* 弧度转换系数
	
	/*前进后退*/
		if(Key.W)
		{
			chas_speed[0] = tardy_start(chas_speed[0],100);
			y_exp = chas_speed[0] * Forward;
		}
		else if(Key.S)
		{
			chas_speed[0] = tardy_start(chas_speed[0],100);
			y_exp = - chas_speed[0] * Forward;
		}
		else 
		{
			y_exp = tardy_stop(y_exp,20);
			chas_speed[0] = 0;
		}
		
		/*左右侧移*/
		if(Key.A)
		{
			chas_speed[1] = tardy_start(chas_speed[1],100);
			x_exp = - chas_speed[1] * Sideways;
		}
		else if(Key.D)
		{
			chas_speed[1] = tardy_start(chas_speed[1],100);
			x_exp = chas_speed[1] * Sideways;
		}
		else 
		{
			x_exp = tardy_stop(x_exp,10);
			chas_speed[1] = 0;
		}

	chas_y = -y_exp*sin(angle_error) - x_exp*cos(angle_error) ;
	chas_x =  y_exp*cos(angle_error) - x_exp*sin(angle_error) ;
//	chas_y =  y_exp*cos(angle_error) - x_exp*sin(angle_error);
//	chas_x =  y_exp*sin(angle_error) + x_exp*cos(angle_error);
	chas_z = 1000;
	
}



