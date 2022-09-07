#include "gimbals.h"
#include "pid.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"
#include "jy901s.h"
#include "coordinate.h"


#include "arm_math.h"

int16_t test_change;

#define Gyro_mode 1  //���������ǻش������ݿ�����̨
#define Motor_mode 3 //���ݵ������Ļ�е�Ƕȿ�����̨

#define ROTATION  1
#define SOLVE     2

extern uint8_t arm_check,arm_error[5];

float p_in,i_in,d_in,p_out,i_out,d_out;
int16_t synchronize_angle;


float coordinate_x = 50.0,coordinate_y = -10.0;

/*��̨����ṹ��*/
Error_position arm1_angle_error;
Error_position arm1_gyro_error;


Error_position arm2_gyro_error1;
Error_position arm2_gyro_error2;
Error_position arm3_gyro_error;
Error_position arm4_gyro_error;
Error_position arm4_angle_error;

int16_t arm1_data,arm2_data;

/**
* @brief  ��������㺯�����ڻ�е��ģʽ�л����ܹ����ݵ�ǰ��е���������������¸�x��y��ֵ
* @param  x�ᡢy������
* @return x�������Ƿ��ڻ�е�ۿɶ���Χ��
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
* @brief  �����жϺ���
* @param  x�ᡢy������
* @return x�������Ƿ��ڻ�е�ۿɶ���Χ��
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
* @brief  �ڶ��ؽ�ͬ������
* @param  ����ϵ��������ͬ��������Ӱ��Ȩ��
* @return ������ֵ
*/
int16_t arm_synchronize(float p)
{
	int16_t deviation;
	
	
	deviation = (Moto_6.turns + Moto_7.turns) *4096+(Moto_6.angle + Moto_7.angle)/2;
	
	deviation = deviation * p;
	return deviation;
}	

/**
* @brief  ��̨�˶����ƺ���
* @param  None
* @return None
*/
void gimbals_controlTask(void const * argument)
{
	/* �����ؽڵ�PID�������*/
	int16_t arm1_exp,arm2_exp[2],arm3_exp,arm4_exp;
	int16_t arm1_out,arm2_out[2],arm3_out,arm4_out;
	
	/* �����ؽڵĻ�е�ǽǶ�ת��ֵ*/
	float arm3_leve_angle,arm3_moto_angle;
	
	/* ���ĹؽڵĻ�е��ת��ֵ*/
	float arm4_real_angle;
	
	/* ���⻷��ֵ*/
	uint8_t loop_scale = 0;
	
	/* ��һ�ؽ���λ*/
	uint8_t arm1_OFF;
	
	/* ���������ж�*/
	uint8_t arm_coordinate;
	
	/* ҡ��ģʽ������ģʽ���л����*/
	uint8_t model;
	
	for(;;)
 {
	 osDelay(1);
	 
	 
	 loop_scale++;
	 
	 /* �жϱ仯��������Ƿ�λ�ڻ�е�ۿɻ����*/
	 if(loop_scale == 5)
	 {
		 /* ���鵵λ�л����ж��Ƿ���Ҫ�̳л�е�۴˿̵�����*/
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
			 /* ����ۼӺ�������Ƿ��ڻ�е�۹����ŵķ�Χ��*/
			 arm_coordinate = arm_determine(coordinate_x,coordinate_y);
			 /* ����ۼӺ�����겻�ڻ�е�۹����ŵķ�Χ�ڣ���ֵ�ټ�ȥ*/
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
		 
		 /* ��һ�ؽ�������㼰PID����*/
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

		 
		 
		 /* ����һ�ؽ���λ*/
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
		 /* �ڶ��ؽ�������㼰PID����*/
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
		 /* �����ؽ�������㼰PID����*/
		 if(loop_scale == 5)
		 {
			 /* ���ֵ����ڻ�е��ˮƽ����Ҫ�ĽǶ�*/
			 arm3_leve_angle = 180.0f - arm2_angle[0];
			 /* ������ڵĻ�е�Ƕ�*/
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


