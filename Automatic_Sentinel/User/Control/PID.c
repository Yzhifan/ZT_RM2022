#include "PID.h"
#include "drv_usart.h"
#include "drv_can.h"

int16_t gyro_exp;
float in_p,in_i,in_d,out_p,out_i,out_d;
int16_t out_number,in_number;
int16_t proportion1,proportion2,proportion;//���⻷�����ٶȱ���

int16_t position_out[4];
/*���̵���ṹ��*/
Error_increment chas1_error;
Error_increment chas2_error;
Error_increment chas3_error;
Error_increment chas4_error;

/*��̨����ṹ��*/
Error_position yaw_angle_error;
Error_position yaw_gyro_error;
Error_position pitch_angle_error;
Error_position pitch_gyro_error;
Error_position yaw_track_error;
Error_position pitch_track_error;
Error_increment pitch_error;

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
	

	if(increment_error->Now_out>=5000)//����������ֻ�� -16384~+16384 ���"C620��ˢ���������ʹ��˵��P32"
	{
		increment_error->Now_out =5000;  //pid�������������ֵ���������ʵ�������������Ϊ������������ڴ����ж϶����޷�
	}
	if(increment_error->Now_out<=-5000)
	{
		increment_error->Now_out= -5000;
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
	if(angle_error->Now_out >5000)
	{
		angle_error->Now_out =5000;
	}
	else if(angle_error->Now_out <-5000)
	{
		angle_error->Now_out =-5000;
	}
	
	/*ʵ��6623����������ʱ��ת���������෴�ģ��ڴ˽�������������һ�����׵Ĵ���*/
	//Gimbal6623_out = -angle_error->Now_out;
	
	return angle_error->Now_out;
}

/**
* @brief  �Զ�����pid�Ƕȼ��㺯��
* @param  λ�����ṹ�壬�Ӿ��������ǣ�����ϵ��p������ϵ��i��΢��ϵ��d
* @return �������
*/
int16_t shoot_Aangle_pid(Error_position* angle_error,int16_t coordinates_error,float kp,float ki,float kd)
{
	
	angle_error->integral +=coordinates_error;//�������ۼ�
	
	angle_error->Now = coordinates_error;
	/*��㲹���������е��ת�������������ͻ������*/
	coordinates_error = zero_point_compensate(coordinates_error,5000,8192);
	
	angle_error->Now_out = kp*coordinates_error + ki*angle_error->integral + kd*(coordinates_error - angle_error->Last);
	
	angle_error->Last = coordinates_error;
	
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
	if(angle_error->Now_out >5000)
	{
		angle_error->Now_out =5000;
	}
	else if(angle_error->Now_out <-5000)
	{
		angle_error->Now_out =-5000;
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
	if(gyro_error->Now_out >30000)
	{
		gyro_error->Now_out =30000;
	}
	else if(gyro_error->Now_out <-30000)
	{
		gyro_error->Now_out =-30000;
	}
  
	/*ʵ��6623����������ʱ��ת���������෴�ģ��ڴ˽�������������һ�����׵Ĵ���*/
	Gimbal6623_out = -gyro_error->Now_out;
	
	return Gimbal6623_out;
}
/**
* @brief  ����pid���㺯��
* @param  �Ƕ����ṹ�壬���ٶ����ṹ�壬�����Ƕȣ����������е�Ƕȣ��������ת��
* @return ���ε������
*/
int16_t Gimbals_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t position_exp,int16_t feedback1,int16_t feedback2)
{
	int16_t position_out1,position_out2;
	
	/*����pid�⻷�ǶȻ�����*/
	position_out1 = Position_Aangle_pid(angle_error,feedback1,position_exp,3,0,0);
	gyro_exp = position_out1;
	/*����pid�ڻ����ٶȻ�����*/
	position_out2 = Position_Gyro_pid(gyro_error,feedback2,position_out1,1.5,0.02,0.3);
	
	if(feedback1 == pitch_angle)
	{
		return position_out2 + 1300;
	}
	else return position_out2; 
}

/**
* @brief  �Զ����ٴ���pid���㺯��
* @param  �Ƕ����ṹ�壬���ٶ����ṹ�壬�����Ƕȣ����������е�Ƕȣ��������ת��
* @return ���ε������
*/
int16_t yaw_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t feedback1,int16_t feedback2)
{
	// static int16_t position_out1,position_out2;
	if(coordinates_x > 550||coordinates_x<-550)
	{
		/*���⻷������������ٶȻ�������ٴΣ��ǶȻ�����һ��*/
		proportion1++;
		if(proportion1 == 3)
		{
			/*����pid�⻷�ǶȻ�����*/
			position_out[0] = shoot_Aangle_pid(angle_error,feedback1,0.6,0,0.15);
			proportion1 = 0;
			
		}
		
		/*����pid�ڻ����ٶȻ�����*/
		position_out[1] = Position_Gyro_pid(gyro_error,feedback2,position_out[0],1.5,0.005,0.3);
	}
	else
	{
		/*���⻷������������ٶȻ�������ٴΣ��ǶȻ�����һ��*/
		proportion1++;
		if(proportion1 == 3)
		{
			/*����pid�⻷�ǶȻ�����*/
			position_out[0] = shoot_Aangle_pid(angle_error,feedback1,0.7,0,0.15);
			proportion1 = 0;
			
		}
		
		/*����pid�ڻ����ٶȻ�����*/
		position_out[1] = Position_Gyro_pid(gyro_error,feedback2,position_out[0],1.8,0.002,0.3);
	}
		
	
	return position_out[1]; 
}

int16_t pitch_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t feedback1,int16_t feedback2)
{
	// static int16_t position_out3,position_out4;
	if(coordinates_y>400||coordinates_y<-400)
	{
		/*���⻷������������ٶȻ�������ٴΣ��ǶȻ�����һ��*/
		proportion2++;
		if(proportion2 == 3)
		{
			/*����pid�⻷�ǶȻ�����*/
			position_out[2] = shoot_Aangle_pid(angle_error,feedback1,0.3,0,0.1);
			proportion2 = 0;
			
		}
		
		/*����pid�ڻ����ٶȻ�����*/
		position_out[3] = Position_Gyro_pid(gyro_error,feedback2,position_out[2],1.2,0.01,0.4);
	}
	else 
	{
				/*���⻷������������ٶȻ�������ٴΣ��ǶȻ�����һ��*/
		proportion2++;
		if(proportion2 == 3)
		{
			/*����pid�⻷�ǶȻ�����*/
			position_out[2] = shoot_Aangle_pid(angle_error,feedback1,0.6,0,0.1);
			proportion2 = 0;
			
		}
		
		/*����pid�ڻ����ٶȻ�����*/
		position_out[3] = Position_Gyro_pid(gyro_error,feedback2,position_out[2],1.6,0.01,0.3);
	}

	
	return position_out[3]+1500; 
}
