#include "pid.h"
#include "drv_usart.h"


int16_t gyro_exp;
float in_p,in_i,in_d,out_p,out_i,out_d;
uint8_t yaw_scale,pitch_scale;//���⻷����ϵ��

/************************************************
   * @brief     ��㲹��
	 * @param[in] ����,�жϹ�������ֵ,������ֵ
   * @retval    ������Ĳ���
 ************************************************/
int16_t zero_point_compensate(int16_t input_param,int16_t threshold,int16_t compensate_param)
{
	if(input_param > threshold)input_param -= compensate_param;
	if(input_param < -threshold)input_param += compensate_param;
	return input_param;
}


/**
* @brief  ����ʽpid���㺯��
* @param  �������ṹ�壬����ٶȣ������ٶȣ�����ϵ��p������ϵ��i��΢��ϵ��d
* @return �������
*/
int16_t Increment_PID(Error_increment *increment_error,int16_t speed_now,int16_t speed_exp,float kp,float ki,float kd) 
{
	increment_error->Now =speed_exp - speed_now;
	
	increment_error->Now_out = kp * (increment_error->Now - increment_error->Last ) //kp��(������� - �ϴ���
	                      + ki * increment_error->Now  //ki�˱������
	                      + kd * (increment_error->Now - 2*increment_error->Last+increment_error->earlier)//kd�ˣ��������-2*�ϴ����+���ϴ���
	                      +increment_error->Last_out;
	

	if(increment_error->Now_out>=16000)//����������ֻ�� -16384~+16384 ���"C620��ˢ���������ʹ��˵��P32"
	{
		increment_error->Now_out =16000;  //pid�������������ֵ���������ʵ�������������Ϊ������������ڴ����ж϶����޷�
	}
	if(increment_error->Now_out<=-16000)
	{
		increment_error->Now_out= -16000;
	}	
	increment_error->Last_out = increment_error->Now_out; //�����������ֵ�����ϴ�������������´�pid����ʹ��
	increment_error->earlier  = increment_error->Last;    //���ϴ���ֵ�������ϴ����������´�pid����ʹ��
	increment_error->Last     = increment_error->Now;     //��������ֵ�����ϴ���
	
	return increment_error->Now_out; //����pid�����ֵ����canͨѶ���͵�������ʹ��
	
}

/**
* @brief  λ��ʽpid�Ƕȼ��㺯��
* @param  λ�����ṹ�壬����Ƕȣ������Ƕȣ�����ϵ��p������ϵ��i��΢��ϵ��d
* @return �������
*/
int16_t Position_Aangle_pid(Error_position* angle_error,int16_t now_angle,int16_t exp_angle,float kp,float ki,float kd)
{
	//int16_t Gimbal6623_out;
	
	angle_error->Now = exp_angle - now_angle; //�������
	angle_error->integral +=angle_error->Now;//�������ۼ�
	
	/*��㲹���������е��ת�������������ͻ������*/
	angle_error->Now = zero_point_compensate(angle_error->Now,5000,8192);
	
	angle_error->Now_out = kp*angle_error->Now + ki*angle_error->integral + kd*(angle_error->Now - angle_error->Last);
	
	angle_error->Last = angle_error->Now;
	
	//�����ֱ���
	if(angle_error->integral >30000)
	{
		angle_error->integral =30000;
	}
	else if(angle_error->integral <-30000)
	{
		angle_error->integral =-30000;
	}

	//�����ֵ
	if(angle_error->Now_out >16000)
	{
		angle_error->Now_out =16000;
	}
	else if(angle_error->Now_out <-16000)
	{
		angle_error->Now_out =-16000;
	}
	
	/*ʵ��6623����������ʱ��ת���������෴�ģ��ڴ˽�������������һ�����׵Ĵ���*/
	//Gimbal6623_out = -angle_error->Now_out;
	
	return angle_error->Now_out;
}


/**
* @brief  λ��ʽpid���ٶȼ��㺯��
* @param  λ�����ṹ�壬����ٶȣ������ٶȣ�����ϵ��p������ϵ��i��΢��ϵ��d
* @return �������
*/
int16_t Position_Gyro_pid(Error_position* gyro_error,int16_t now_velocity,int16_t exp_velocity,float kp,float ki,float kd)
{
	int16_t Gimbal6623_out;
	
	gyro_error->Now = exp_velocity - now_velocity; //�������
	gyro_error->integral +=gyro_error->Now;//�������ۼ�
	

	gyro_error->Now_out = kp*gyro_error->Now + ki*gyro_error->integral + kd*(gyro_error->Now - gyro_error->Last);
	
	gyro_error->Last = gyro_error->Now;
	
	//�����ֱ���
	if(gyro_error->integral >30000)
	{
		gyro_error->integral =30000;
	}
	else if(gyro_error->integral <-30000)
	{
		gyro_error->integral =-30000;
	}

	//�����ֵ
	if(gyro_error->Now_out >16000)
	{
		gyro_error->Now_out =16000;
	}
	else if(gyro_error->Now_out <-16000)
	{
		gyro_error->Now_out =-16000;
	}
  
	/*ʵ��6623����������ʱ��ת���������෴�ģ��ڴ˽�������������һ�����׵Ĵ���*/
	Gimbal6623_out = -gyro_error->Now_out;
	
	return Gimbal6623_out;
}
/**
* @brief  ��̨ƫ���Ǵ���pid���㺯��
* @param  �Ƕ����ṹ�壬���ٶ����ṹ�壬�����Ƕȣ����������е�Ƕȣ��������ת��
* @return ���ε������
*/
int16_t yaw_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t position_exp,int16_t feedback1,int16_t feedback2)
{
	static int16_t position_out1,position_out2;
	yaw_scale++;
	if(yaw_scale == 3)
	{
		/*����pid�⻷�ǶȻ�����*/
		position_out1 = Position_Aangle_pid(angle_error,feedback1,position_exp,1.2,out_i,0.2);
		yaw_scale = 0;
	}
	

	/*����pid�ڻ����ٶȻ�����*/
	position_out2 = Position_Gyro_pid(gyro_error,feedback2,position_out1,1.8,0.02,0.3);
	
	return position_out2; 
}
/**
* @brief  ��̨�����Ǵ���pid���㺯��
* @param  �Ƕ����ṹ�壬���ٶ����ṹ�壬�����Ƕȣ����������е�Ƕȣ��������ת��
* @return ���ε������
*/
int16_t pitch_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t position_exp,int16_t feedback1,int16_t feedback2)
{
	static int16_t position_out1,position_out2;
	pitch_scale++;
	
	if(pitch_scale == 3)
	{
		/*����pid�⻷�ǶȻ�����*/
		position_out1 = Position_Aangle_pid(angle_error,feedback1,position_exp,2,0,0);
		pitch_scale = 0;
	}


	/*����pid�ڻ����ٶȻ�����*/
	position_out2 = Position_Gyro_pid(gyro_error,feedback2,position_out1,2,0.03,0.3);
	
	return position_out2 -800; 
}

