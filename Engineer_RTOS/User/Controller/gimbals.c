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


/*��̨����ṹ��*/
Error_position arm1_gyro_error;
Error_position arm2_gyro_error1;
Error_position arm2_gyro_error2;

/* ��̨���������Ϊ�������֣���һ�ؽں͵ڶ��ؽڡ������ؽڡ����Ĺؽ���ת���
	 ��һ���֣��ɵ�һ�ؽں͵ڶ��ؽھ����������㺯��������������ǽǶ���ɻ�е����չ��x��y��������
	 �ڶ����֣��ɵ����ؽ�ͨ��������ڶ��ؽڵ���ԽǶ������ֵ����ؽڵ�ˮƽ������һ���ֵ���λ�ƺ󣬽��ˮƽ����
						����pid���Ƶ��ת�٣��˹����ڵ����ؽڵĽǶ�
	 �������֣�
*/

/**
* @brief  ��̨�˶����ƺ���
* @param  None
* @return None
*/
void gimbals_controlTask(void const * argument)
{
	int16_t arm1_exp,arm2_exp[2],arm1_out,arm2_out[2];
	uint8_t loop_scale = 0;
	
	for(;;)
 {
	 osDelay(1);
	 
	 loop_scale++;
	 
	 if(arm_error[0] == HAL_OK)
	 {
		 
		 if(loop_scale == 5)
		 {
			 arm1_exp = Arm_Out1(50,30,arm1_angle[0],10);
			 
		 }
		 arm1_out = Position_Gyro_pid(&arm1_gyro_error,Moto_5.speed,arm1_exp,0,0,0);
		 Can_SendMoto_Arm(arm1_out,MOTO_ID_5);
	 }
	 if((arm_error[1]&&arm_error[2]) == HAL_OK)
	 {
		 if(loop_scale == 5)
		 {
			arm2_exp[0] = Arm_Out2(50,30,arm2_angle[0],20);
			arm2_exp[1] = Arm_Out2(50,30,arm2_angle[0],20)+0;
		 }
		 arm2_out[0] = Position_Gyro_pid(&arm2_gyro_error1,Moto_6.speed,arm2_exp[0],0,0,0);
		 arm2_out[1] = Position_Gyro_pid(&arm2_gyro_error2,Moto_7.speed,arm2_exp[1],0,0,0);
	 }
		 
	 
	 
	
		if(loop_scale == 5)
		{
			loop_scale = 0;
		}
 }

}


