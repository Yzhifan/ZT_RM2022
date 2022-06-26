#include "gimbals.h"
#include "pid.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"
#include "jy901s.h"

#define Gyro_mode 1  //���������ǻش������ݿ�����̨
#define Motor_mode 3 //���ݵ������Ļ�е�Ƕȿ�����̨


/*��̨����ṹ��*/
Error_position yaw_angle_error;
Error_position yaw_gyro_error;
Error_position pitch_angle_error;
Error_position pitch_gyro_error;



/**
* @brief  ��̨�˶����ƺ���
* @param  None
* @return None
*/
void gimbals_control(void)
{
	int16_t yaw_out,pitch_out;
	
	/*������ģʽ*/
	if(RC.s1 == Gyro_mode)
	{
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);
		
		yaw_out = yaw_loops_pid(&yaw_angle_error,&yaw_gyro_error,-Mouse_move.x,yaw_gyro_angle,yaw_speed);
		pitch_out = pitch_loops_pid(&pitch_angle_error,&pitch_gyro_error,Mouse_move.y,pitch_angle,pitch_speed);
	}
	/*�����е��ģʽ*/
	else if(RC.s1 == Motor_mode)
	{
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_SET);
		
		yaw_out   = yaw_loops_pid(&yaw_angle_error,&yaw_gyro_error,-500,yaw_angle,yaw_speed);
    pitch_out = pitch_loops_pid(&pitch_angle_error,&pitch_gyro_error,350,pitch_angle,pitch_speed);
	}
	/*��̨�ϵ�*/
	else 
		
	{
		HAL_GPIO_WritePin(GPIOI,GPIO_PIN_7,GPIO_PIN_RESET);
		yaw_out = 0;
		pitch_out = 0;
	}
	
	Can_SendMoto(yaw_out,MOTO_ID_8);
	Can_SendMoto_Gimbals(pitch_out,MOTO_ID_9);

}

//void yaw_test(void)
//{
//	int16_t out;
//	
////	out = Position_Aangle_pid(&yaw_angle_error,test_angle,Mouse_move.x,out_p,out_i,out_d);
////	out = Position_Gyro_pid(&yaw_gyro_error,gyro[2],-Mouse.x,in_p,in_i,in_d);
//	out = pitch_loops_pid(&yaw_angle_error,&yaw_gyro_error,Mouse_move.x,pitch_angle,Moto_6.speed);

////	out = Position_Aangle_pid(&yaw_angle_error,yaw_angle,Mouse_move.x,10,0.1,3); //1000HZ  M6020���
////	out = Position_Gyro_pid(&yaw_gyro_error,gyro_real,-Mouse.x,12,0.2,2);///���ٶȿ��ò��� 1000HZ  M6020���
////	out = Position_Gyro_pid(&yaw_gyro_error,gyro_real,-Mouse.x,8,0.5,2);///���ٶȿ��ò���   200HZ M6020���
////	out = pitch_loops_pid(&yaw_angle_error,&yaw_gyro_error,Mouse_move.x,test_angle,gyro[2]);//out_p 5,in_p 5,in_i 0.5,in_d 1,200HZ,M6020���
////	Can_SendMoto_gimbals(out,MOTO_ID_6);
//}

