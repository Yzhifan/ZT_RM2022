#include "pid.h"
#include "drv_usart.h"


int16_t gyro_exp;
float in_p,in_i,in_d,out_p,out_i,out_d;
uint8_t yaw_scale,pitch_scale;//内外环比例系数

/************************************************
   * @brief     零点补偿
	 * @param[in] 参数,判断过零点的阈值,补偿数值
   * @retval    补偿后的参数
 ************************************************/
int16_t zero_point_compensate(int16_t input_param,int16_t threshold,int16_t compensate_param)
{
	if(input_param > threshold)input_param -= compensate_param;
	if(input_param < -threshold)input_param += compensate_param;
	return input_param;
}


/**
* @brief  增量式pid计算函数
* @param  增量误差结构体，电机速度，期望速度，比例系数p，积分系数i，微分系数d
* @return 本次输出
*/
int16_t Increment_PID(Error_increment *increment_error,int16_t speed_now,int16_t speed_exp,float kp,float ki,float kd) 
{
	increment_error->Now =speed_exp - speed_now;
	
	increment_error->Now_out = kp * (increment_error->Now - increment_error->Last ) //kp乘(本次误差 - 上次误差）
	                      + ki * increment_error->Now  //ki乘本次误差
	                      + kd * (increment_error->Now - 2*increment_error->Last+increment_error->earlier)//kd乘（本次误差-2*上次误差+上上次误差）
	                      +increment_error->Last_out;
	

	if(increment_error->Now_out>=16000)//电机电流最多只能 -16384~+16384 详见"C620无刷电机调速器使用说明P32"
	{
		increment_error->Now_out =16000;  //pid输出如果超过这个值将超出电机实际输出电流，称为数据溢出，故在此做判断对其限幅
	}
	if(increment_error->Now_out<=-16000)
	{
		increment_error->Now_out= -16000;
	}	
	increment_error->Last_out = increment_error->Now_out; //将本次输出赋值给“上次输出”，留到下次pid运算使用
	increment_error->earlier  = increment_error->Last;    //将上次误差赋值给“上上次误差”，留到下次pid运算使用
	increment_error->Last     = increment_error->Now;     //将本次误差赋值给“上次误差”
	
	return increment_error->Now_out; //返回pid运算的值，供can通讯发送电流函数使用
	
}

/**
* @brief  位置式pid角度计算函数
* @param  位置误差结构体，电机角度，期望角度，比例系数p，积分系数i，微分系数d
* @return 本次输出
*/
int16_t Position_Aangle_pid(Error_position* angle_error,int16_t now_angle,int16_t exp_angle,float kp,float ki,float kd)
{
	//int16_t Gimbal6623_out;
	
	angle_error->Now = exp_angle - now_angle; //本次误差
	angle_error->integral +=angle_error->Now;//误差积分累加
	
	/*零点补偿，解决机械角转过零点带来的误差突变问题*/
	angle_error->Now = zero_point_compensate(angle_error->Now,5000,8192);
	
	angle_error->Now_out = kp*angle_error->Now + ki*angle_error->integral + kd*(angle_error->Now - angle_error->Last);
	
	angle_error->Last = angle_error->Now;
	
	//抗积分饱和
	if(angle_error->integral >30000)
	{
		angle_error->integral =30000;
	}
	else if(angle_error->integral <-30000)
	{
		angle_error->integral =-30000;
	}

	//输出阈值
	if(angle_error->Now_out >16000)
	{
		angle_error->Now_out =16000;
	}
	else if(angle_error->Now_out <-16000)
	{
		angle_error->Now_out =-16000;
	}
	
	/*实测6623给正电流的时候转动方向是相反的，在此将处理后的数据做一个简易的处理*/
	//Gimbal6623_out = -angle_error->Now_out;
	
	return angle_error->Now_out;
}


/**
* @brief  位置式pid角速度计算函数
* @param  位置误差结构体，电机速度，期望速度，比例系数p，积分系数i，微分系数d
* @return 本次输出
*/
int16_t Position_Gyro_pid(Error_position* gyro_error,int16_t now_velocity,int16_t exp_velocity,float kp,float ki,float kd)
{
	int16_t Gimbal6623_out;
	
	gyro_error->Now = exp_velocity - now_velocity; //本次误差
	gyro_error->integral +=gyro_error->Now;//误差积分累加
	

	gyro_error->Now_out = kp*gyro_error->Now + ki*gyro_error->integral + kd*(gyro_error->Now - gyro_error->Last);
	
	gyro_error->Last = gyro_error->Now;
	
	//抗积分饱和
	if(gyro_error->integral >30000)
	{
		gyro_error->integral =30000;
	}
	else if(gyro_error->integral <-30000)
	{
		gyro_error->integral =-30000;
	}

	//输出阈值
	if(gyro_error->Now_out >16000)
	{
		gyro_error->Now_out =16000;
	}
	else if(gyro_error->Now_out <-16000)
	{
		gyro_error->Now_out =-16000;
	}
  
	/*实测6623给正电流的时候转动方向是相反的，在此将处理后的数据做一个简易的处理*/
	Gimbal6623_out = -gyro_error->Now_out;
	
	return Gimbal6623_out;
}
/**
* @brief  云台偏航角串级pid运算函数
* @param  角度误差结构体，角速度误差结构体，期望角度，电机反馈机械角度，电机反馈转速
* @return 本次电流输出
*/
int16_t yaw_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t position_exp,int16_t feedback1,int16_t feedback2)
{
	static int16_t position_out1,position_out2;
	yaw_scale++;
	if(yaw_scale == 3)
	{
		/*串级pid外环角度环计算*/
		position_out1 = Position_Aangle_pid(angle_error,feedback1,position_exp,1.2,out_i,0.2);
		yaw_scale = 0;
	}
	

	/*串级pid内环角速度环计算*/
	position_out2 = Position_Gyro_pid(gyro_error,feedback2,position_out1,1.8,0.02,0.3);
	
	return position_out2; 
}
/**
* @brief  云台俯仰角串级pid运算函数
* @param  角度误差结构体，角速度误差结构体，期望角度，电机反馈机械角度，电机反馈转速
* @return 本次电流输出
*/
int16_t pitch_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t position_exp,int16_t feedback1,int16_t feedback2)
{
	static int16_t position_out1,position_out2;
	pitch_scale++;
	
	if(pitch_scale == 3)
	{
		/*串级pid外环角度环计算*/
		position_out1 = Position_Aangle_pid(angle_error,feedback1,position_exp,2,0,0);
		pitch_scale = 0;
	}


	/*串级pid内环角速度环计算*/
	position_out2 = Position_Gyro_pid(gyro_error,feedback2,position_out1,2,0.03,0.3);
	
	return position_out2 -800; 
}

