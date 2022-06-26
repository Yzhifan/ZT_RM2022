#include "gimbals.h"
#include "PID.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"

#define Left  0 //����Ѳ��
#define Right 1 //����Ѳ��

#define Drop  0 //�½�
#define Rise  1 //����

#define Cruise 0 //Ѳ��
#define Track  1 //����


extern float in_p,in_i,in_d,out_p,out_i,out_d;

int8_t tims;
int16_t yaw_exp,pitch_exp;
uint8_t turn_sign,lift_sign;//ת���־λ��̧����־λ
uint8_t Gimbals_Mode;//��̨ģʽ��־(Ѳ��ģʽ������ģʽ�л�)
int16_t yaw_position_out,pitch_position_out;


/**
* @brief  ��̨Ѳ��������ͨ���ۼ�ֵ����̨���ض����ٶ�Ѳ����Χ
* @param  None
* @return None
*/
void gimbals_cruise(void)
{
	/*�ж����¸�����־λ��״̬*/
	if(lift_sign == Rise)
	{
		/*�ж�pitch��Ƕ��Ƿ񵽴���λ������ı丩����־λ��ת��*/
		if(pitch_exp >= 300 )
		{
			lift_sign = Drop;
		}
		/*�����pitch�������Ƕ�ֵ�����ۼ�*/
		else pitch_exp+=2;
	}
	else if(lift_sign == Drop)
	{
		
		if(pitch_exp <= -300)
		{
			lift_sign = Rise;
		}
		else pitch_exp-=2;
	}
	/*�ж�����ת���־λ��״̬*/
	if(turn_sign == Right)
	{
		/*�ж�yaw��Ƕ��Ƿ񵽴���λ������ı�ת���־λ��ת��*/
		if(yaw_exp >=-1600)
		{
			turn_sign = Left;
		}
		else yaw_exp+=6;
	}
	else if(turn_sign == Left)
	{
		
		if(yaw_exp<=-2800)
		{
			turn_sign = Right;
		}
		else yaw_exp-=6;
	}
}


/**
* @brief  ��̨���ƺ���
* @param  None
* @return None
*/
void gimbals_control(void)
{
	int16_t yaw_out,pitch_out;
	
	/*����״̬��ѭ��״̬�ж�*/
 switch(recognize)
 {	
	 /*ʶ���־λΪ0������̨�л���Ѳ��ģʽ*/
	 case 0:
	 {
		 Gimbals_Mode = Cruise;
		 

	 }break;
	 /*���ʶ���־λ�Ƿ�Ϊ1��������̨ģʽת��Ϊ����ģʽ*/
	 case 1:
	 {
		 Gimbals_Mode = Track;
		/*ʹ��̨�ӽ�������ģʽ��Ļ�е�Ƕȿ�ʼѲ��*/
		 pitch_exp = pitch_angle;
		 yaw_exp = yaw_angle;
	 }break;
	

 }
 
 /*�ж���̨����ģʽ��������̨��ģʽѡ���Ӧ��pid��������*/
	if(Gimbals_Mode == Cruise)
	{
		gimbals_cruise();
		yaw_out   = Gimbals_loops_pid(&yaw_angle_error,&yaw_gyro_error,yaw_exp,yaw_angle,yaw_speed);
		pitch_out = Gimbals_loops_pid(&pitch_angle_error,&pitch_gyro_error,pitch_exp,pitch_angle,pitch_speed);
	}
	 else if(Gimbals_Mode == Track)
	{

		yaw_out = yaw_loops_pid(&yaw_track_error,&yaw_gyro_error,coordinates_x,yaw_speed);
		pitch_out = pitch_loops_pid(&pitch_track_error,&pitch_gyro_error,coordinates_y,pitch_speed);
		
	}
	else
	{
		yaw_out = 0;
		pitch_out = 0;
	}

	Can_SendMoto_gimbals(yaw_out,MOTO_ID_5);
	Can_SendMoto_gimbals(pitch_out,MOTO_ID_6);
	
}



