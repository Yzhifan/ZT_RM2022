#include "gimbals.h"
#include "pid.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"
#include "jy901s.h"
#include "coordinate.h"


#include "arm_math.h"

int16_t test_change;

#define Gyro_mode 1  //根据陀螺仪回传的数据控制云台
#define Motor_mode 3 //根据电机自身的机械角度控制云台

#define ROTATION  1
#define SOLVE     2

extern uint8_t arm_check,arm_error[5];

float p_in,i_in,d_in,p_out,i_out,d_out;
int16_t synchronize_angle;


float coordinate_x = 50.0,coordinate_y = -10.0;

/*云台电机结构体*/
Error_position arm1_angle_error;
Error_position arm1_gyro_error;


Error_position arm2_gyro_error1;
Error_position arm2_gyro_error2;
Error_position arm3_gyro_error;
Error_position arm4_gyro_error;
Error_position arm4_angle_error;

int16_t arm1_data,arm2_data;

/**
* @brief  坐标逆解算函数，在机械臂模式切换后能够根据当前机械臂所处的坐标重新给x，y赋值
* @param  x轴、y轴坐标
* @return x轴坐标是否在机械臂可动范围内
*/

void arm_now_coordi(float arm1_angle,float arm2_angle,float* x,float* y)
{
	float x1,x2,y1,y2;
	
	x1 = arm_cos_f32(arm1_angle*180.0f/Pi)*33.8f;
	y1 = arm_sin_f32(arm1_angle*180.0f/Pi)*33.8f;
	
	
	x2 = arm_cos_f32(arm2_angle*180.0f/Pi)*33.0f;
	y2 = arm_sin_f32(arm2_angle*180.0f/Pi)*33.0f;
	
	*x = x1 + x2;
	*y = y1 + y2;
}

/**
* @brief  坐标判断函数
* @param  x轴、y轴坐标
* @return x轴坐标是否在机械臂可动范围内
*/

uint8_t arm_determine(float x,float y)
{
	float angle_limit[3];
	float temp[3];
	
	temp[0] = (x - 23.56f)*(x - 23.56f) + (y - 24.23f)* (y - 24.23f);
	
	angle_limit[0] = sqrt(temp[0]);
	
	temp[1] = (x - 33.67f)*(x - 33.67f) + (y + 2.95f) * (y + 2.95f);
	
	angle_limit[1] = sqrt(temp[1]);
	
	temp[2] = x * x + y * y;
	
	angle_limit[2] = sqrt(temp[2]);
	
	if(angle_limit[0]>33&&angle_limit[1]<=33)
	{
		return HAL_OK;
	}
	else if(angle_limit[0]>33&angle_limit[1]>=33&&angle_limit[2]<66.8f)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}
	
}

/**
* @brief  第二关节同步函数
* @param  比例系数，调节同步函数的影响权重
* @return 纠正数值
*/
int16_t arm_synchronize(float p)
{
	int16_t deviation;
	
	
	deviation = (Moto_6.turns + Moto_7.turns) *4096+(Moto_6.angle + Moto_7.angle)/2;
	
	deviation = deviation * p;
	return deviation;
}	

/**
* @brief  云台运动控制函数
* @param  None
* @return None
*/
void gimbals_controlTask(void const * argument)
{
	/* 各个关节的PID运算输出*/
	int16_t arm1_exp,arm2_exp[2],arm3_exp,arm4_exp;
	int16_t arm1_out,arm2_out[2],arm3_out,arm4_out;
	
	/* 第三关节的机械角角度转换值*/
	float arm3_leve_angle,arm3_moto_angle;
	
	/* 第四关节的机械角转换值*/
	float arm4_real_angle;
	
	/* 内外环比值*/
	uint8_t loop_scale = 0;
	
	/* 第一关节限位*/
	uint8_t arm1_OFF;
	
	/* 解算坐标判断*/
	uint8_t arm_coordinate;
	
	/* 摇臂模式、解算模式的切换检测*/
	uint8_t model;
	
	for(;;)
 {
	 osDelay(1);
	 
	 
	 loop_scale++;
	 
	 /* 判断变化后的坐标是否位于机械臂可活动区间*/
	 if(loop_scale == 5)
	 {
		 /* 检验档位切换，判断是否需要继承机械臂此刻的坐标*/
		 if(RC.s1 == 1&&model!=SOLVE)
		 {
			 model = SOLVE;
			 
			 arm_now_coordi(arm1_angle[0],arm2_angle[0],&coordinate_x,&coordinate_y);
		 }
		 else if(RC.s1 == 3&&model!=ROTATION)
		 {
			 model = ROTATION;
		 }
		if(RC.y >200)
		{
			 
			 coordinate_x+=0.02f;
			 /* 检测累加后的坐标是否在机械臂够得着的范围内*/
			 arm_coordinate = arm_determine(coordinate_x,coordinate_y);
			 /* 如果累加后的坐标不在机械臂够得着的范围内，将值再减去*/
			 if(arm_coordinate == HAL_ERROR)
			 {
				 coordinate_x-=0.02f; 
			 }
		}
	  else if(RC.y < -200)
	  {
		  coordinate_x-= 0.02f;
		  arm_coordinate = arm_determine(coordinate_x,coordinate_y);
		 if(arm_coordinate == HAL_ERROR)
		 {
			 coordinate_x+=0.02f;
		 }
	 }
	 if(RC.pitch > 200)
		 {
			 coordinate_y +=0.02f;
			 arm_coordinate = arm_determine(coordinate_x,coordinate_y);
			if(arm_coordinate == HAL_ERROR)
			 {
				 coordinate_y -=0.02f;
			 }
		 }
			 else if(RC.pitch < -200)
			 {
				 coordinate_y -=0.02f;
				 arm_coordinate = arm_determine(coordinate_x,coordinate_y);
				 if(arm_coordinate == HAL_ERROR)
				 {
					 coordinate_y +=0.02f;
				 }
			 }
	 }
	 
	 
	 if(arm_error[0] == HAL_OK)
	 {
		 arm1_OFF = HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12);
		 
		 /* 第一关节坐标解算及PID运算*/
		 if(loop_scale == 5)
		 {
			 arm1_exp = Arm_Out1(coordinate_x,coordinate_y,arm1_angle[0],400);

		 }
		 if(RC.s1 == 1)
		 {
			 if(arm1_exp > 6500)
			 {
				 arm1_exp = 6500;
			 }
			 else if(arm1_exp < -6500)
			 {
				 arm1_exp = -6500;
			 }
			 arm1_out = Position_Gyro_pid(&arm1_gyro_error,Moto_5.speed,arm1_exp,5,0.1,1);
		 }
		 else if(RC.s1 == 3)
		 {
			 arm1_out = Position_Gyro_pid(&arm1_gyro_error,Moto_5.speed,-RC.y*2,5,0.1,1);
		 }
		 else arm1_out =0;

		 
		 
		 /* 检查第一关节限位*/
		 if(arm1_OFF == HAL_OK&&arm1_out>0)
		 {
			 arm1_out = 0;
			 Moto_5.turns = 0;
		 }
		 else if(Moto_5.turns < -260&&arm1_out<0)
		 {
			 arm1_out = 0;
		 }
		 Can_SendMoto_Arm(arm1_out,MOTO_ID_5);
	 }
	 if((arm_error[1]&&arm_error[2]) == HAL_OK)
	 {
		 /* 第二关节坐标解算及PID运算*/
		 if(loop_scale == 5)
		 {
			 if(RC.pitch > 200)
			 {
				 coordinate_y +=0.02f;
				 arm_coordinate = arm_determine(coordinate_x,coordinate_y);
				  if(arm_coordinate == HAL_ERROR)
				 {
					 coordinate_y -=0.02f;
				 }
			 }
			 else if(RC.pitch < -200)
			 {
				 coordinate_y -=0.02f;
				 arm_coordinate = arm_determine(coordinate_x,coordinate_y);
				  if(arm_coordinate == HAL_ERROR)
				 {
					 coordinate_y +=0.02f;
				 }
			 }
			arm2_exp[0] = Arm_Out2(coordinate_x,coordinate_y,arm2_angle[0],30);
			arm2_exp[1] = -arm2_exp[0]-arm_synchronize(0.2);
			 
			 synchronize_angle = arm_synchronize(0.2);
		 }
		 if(RC.s1 == 1)
		 {
			arm2_out[0] = Position_Gyro_pid(&arm2_gyro_error1,Moto_6.speed,arm2_exp[0],10,0.1,1);
			arm2_out[1] = Position_Gyro_pid(&arm2_gyro_error2,Moto_7.speed,arm2_exp[1],10,0.1,1);
		 }
		 else if(RC.s1 == 3)
		 {
			arm2_out[0] = Position_Gyro_pid(&arm2_gyro_error1,Moto_6.speed,RC.pitch,10,0.1,1);
			arm2_out[1] = Position_Gyro_pid(&arm2_gyro_error2,Moto_7.speed,-RC.pitch -synchronize_angle,10,0.1,1);
		 }
		 else 
		 {
			 arm2_out[0] = 0;
			 arm2_out[1] = 0;
		 }
		 Can_SendMoto_Arm(arm2_out[0],MOTO_ID_6);
		 Can_SendMoto_Arm(arm2_out[1],MOTO_ID_7);
	 }
	 else 
	 {
		 Can_SendMoto_Arm(0,MOTO_ID_6);
		 Can_SendMoto_Arm(0,MOTO_ID_7);
	 }
	 if(arm_error[3] == HAL_OK)
	 {
		 /* 第三关节坐标解算及PID运算*/
		 if(loop_scale == 5)
		 {
			 /* 保持第三节机械臂水平所需要的角度*/
			 arm3_leve_angle = 180.0f - arm2_angle[0];
			 /* 电机现在的机械角度*/
			 arm3_moto_angle = -(Moto_8.turns + (float)Moto_8.angle/8192) * (360.0/870.0);
			 
			 arm3_exp = (arm3_leve_angle - arm3_moto_angle) * 100;
			 
		 }
		 arm3_out = Position_Gyro_pid(&arm3_gyro_error,Moto_8.speed,RC.f*3,10,0,1);
		 
		 Can_SendMoto_Arm(arm3_out,MOTO_ID_8);
	 }
	 else {
	  Can_SendMoto_Arm(0,MOTO_ID_8);
	 }
	 if(arm_error[4] == HAL_OK)
	 {
		 if(loop_scale == 5)
		 {
			 arm4_real_angle = (float)Moto_9.angle*10.0f/8192 + Moto_9.turns*10;
			 
			 arm4_exp = Position_Aangle_pid(&arm4_angle_error,arm4_real_angle,0,p_out,0,0);
		 }
		 arm4_out = Position_Gyro_pid(&arm4_gyro_error,Moto_9.speed,arm4_exp,10,0,1);
		 
		 Can_SendMoto_Arm2(arm4_out,MOTO_ID_9);
	 }
	 else {
	  Can_SendMoto_Arm2(0,MOTO_ID_9);
	 }

		 
		if(loop_scale == 5)
		{
			loop_scale = 0;
		}
 }

}


