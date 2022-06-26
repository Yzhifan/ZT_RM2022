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

extern float in_p,in_i,in_d,out_p,out_i,out_d;

uint8_t gimbals_scale;
int16_t yaw,pitch;

/**
* @brief  ��̨�˶����ƺ���
* @param  None
* @return None
*/
void gimbals_control(void)
{
	/*����yaw��ļ���Ƶ�ʱ�pitch���3�����ֽ����߷�����һ�������ڣ�������þ�̬��������pitch��ÿ���ε������һ��*/
	static int16_t yaw_out,pitch_out;
	
	/*������ģʽ*/
	if(RC.s1 == Gyro_mode)
	{
		yaw_out = yaw_loops_pid(&yaw_angle_error,&yaw_gyro_error,-Mouse_move.x,yaw_gyro_angle,yaw_gyro);
		if(gimbals_scale == 3)
		{
			pitch_out = pitch_loops_pid(&pitch_angle_error,&pitch_gyro_error,-Mouse_move.y,pitch_gyro_angle,pitch_gyro);
			gimbals_scale = 0;
			
		}
		else gimbals_scale++;
		
	}
	/*��̨�ϵ�*/
	else 
	{
		yaw_out = 0;
		pitch_out = 0;
	}

	Can_SendMoto_Gimbals(yaw_out,MOTO_ID_5);
	Can_SendMoto_Gimbals(pitch_out,MOTO_ID_6);

}


