#include "gimbals.h"
#include "pid.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"
#include "jy901s.h"
#include "coordinate.h"

#define Gyro_mode 1  //���������ǻش������ݿ�����̨
#define Motor_mode 3 //���ݵ������Ļ�е�Ƕȿ�����̨

extern uint8_t arm_check,arm_error[5];

float p_in,i_in,d_in,p_out,i_out,d_out;
int16_t exp_angle;

/*��̨����ṹ��*/
Error_increment arm1_speed_error;
Error_position arm1_angle_error;
Error_position arm1_gyro_error;


Error_position arm2_gyro_error1;
Error_position arm2_gyro_error2;
Error_position arm3_gyro_error;

/* ��̨���������Ϊ�������֣���һ�ؽں͵ڶ��ؽڡ������ؽڡ����Ĺؽ���ת���
	 ��һ���֣��ɵ�һ�ؽں͵ڶ��ؽھ����������㺯��������������ǽǶ���ɻ�е����չ��x��y��������
	 �ڶ����֣��ɵ����ؽ�ͨ��������ڶ��ؽڵ���ԽǶ������ֵ����ؽڵ�ˮƽ������һ���ֵ���λ�ƺ󣬽��ˮƽ����
						����pid���Ƶ��ת�٣��˹����ڵ����ؽڵĽǶ�
	 �������֣�
*/


/**
* @brief  �ڶ��ؽ�ͬ������
* @param  ����ϵ��������ͬ��������Ӱ��Ȩ��
* @return ������ֵ
*/
int16_t arm_synchronize(float p)
{
	int16_t deviation;
	
	
	deviation = (Moto_6.turns - Moto_7.turns) *4096+(Moto_6.angle - Moto_7.angle)/2;
	
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
	int16_t arm1_exp,arm2_exp[2],arm3_exp,arm1_out,arm2_out[2],arm3_out;
	float arm3_leve_angle,arm3_moto_angle;
	uint8_t loop_scale = 0;
	
	for(;;)
 {
	 osDelay(1);
	 
	 loop_scale++;
	 
	 if(arm_error[0] == HAL_OK)
	 {
		 /* ��һ�ؽ�������㼰PID����*/
		 if(loop_scale == 5)
		 {
			 arm1_exp = Arm_Out1(50,30,arm1_angle[0],10);
//			 arm1_exp = Position_Aangle_pid(&arm1_angle_error,Moto_5.angle,exp_angle,p_out,i_out,d_out);
		 }
#if 1
		 arm1_out = Position_Gyro_pid(&arm1_gyro_error,Moto_5.speed,arm1_exp,p_in,i_in,d_in);
#else
		 arm1_out = Increment_PID(&arm1_speed_error,Moto_5.speed,arm1_exp,p_in,i_in,d_in);
 #endif
		 
//		 Can_SendMoto_Arm(arm1_out,MOTO_ID_5);
	 }
	 if((arm_error[1]&&arm_error[2]) == HAL_OK)
	 {
		 /* �ڶ��ؽ�������㼰PID����*/
		 if(loop_scale == 5)
		 {
			arm2_exp[0] = Arm_Out2(50,30,arm2_angle[0],20);
			arm2_exp[1] = Arm_Out2(50,30,arm2_angle[0],20)+arm_synchronize(0.1);
			 
			 exp_angle = arm_synchronize(0.05);
		 }
		 arm2_out[0] = Position_Gyro_pid(&arm2_gyro_error1,Moto_6.speed,arm2_exp[0],0,0,0);
		 arm2_out[1] = Position_Gyro_pid(&arm2_gyro_error2,Moto_7.speed,arm2_exp[1],0,0,0);
	 }
	 if(arm_error[3] == HAL_OK)
	 {
		 /* �����ؽ�������㼰PID����*/
		 if(loop_scale == 5)
		 {
			 /* ���ֵ����ڻ�е��ˮƽ����Ҫ�ĽǶ�*/
			 arm3_leve_angle = 180.0 - arm2_angle[0];
			 /* ������ڵĻ�е�Ƕ�*/
			 arm3_moto_angle = (Moto_8.turns + (float)Moto_8.angle/8192) * (360.0/290.0);
			 
			 arm3_exp = (arm3_leve_angle - arm3_moto_angle) * 100;
			 
			 if(arm3_exp > 4000)
			 {
				 arm3_exp = 4000;
			 }
			
		 }
		 arm3_out = Position_Gyro_pid(&arm3_gyro_error,Moto_8.speed,arm3_exp,0,0,0);
		 
	 }
		 
	 
	 
	
		if(loop_scale == 5)
		{
			loop_scale = 0;
		}
 }

}


