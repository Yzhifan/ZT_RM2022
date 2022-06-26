#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "stm32f4xx_hal.h"

/*增量pid运算误差结构体*/
typedef struct
{
	int16_t Now;//本次误差
	int16_t Last;//上次误差
	int16_t earlier;//上上次误差
	int16_t Last_out; //上次的输出
	int16_t Now_out; //本次输出
	
	
}Error_increment; 


/*位置pid运算误差结构体*/
typedef struct
	
{
	int16_t Now;//本次误差
	int16_t Last;//上次误差
	int16_t integral;//误差积分累加
	int16_t Now_out; //本次输出
	
}Error_position;


/*底盘电机结构体*/
extern Error_increment chas1_error;
extern Error_increment chas2_error;
extern Error_increment chas3_error;
extern Error_increment chas4_error;


/*云台电机结构体*/
extern Error_position yaw_angle_error;
extern Error_position yaw_gyro_error;
extern Error_position pitch_angle_error;
extern Error_position pitch_gyro_error;
extern Error_position yaw_track_error;
extern Error_position pitch_track_error;
extern Error_increment pitch_error;


extern int16_t P_data,D_data;

int16_t Increment_PID(Error_increment *increment_error,int16_t speed_now,int16_t speed_exp,float kp,float ki,float kd); 
int16_t Position_Aangle_pid(Error_position* angle_error,int16_t now_angle,int16_t exp_angle,float kp,float ki,float kd);
int16_t Position_Gyro_pid(Error_position* gyro_error,int16_t now_velocity,int16_t exp_velocity,float kp,float ki,float kd);
int16_t Gimbals_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t position_exp,int16_t feedback1,int16_t feedback2);
int16_t yaw_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t feedback1,int16_t feedback2);
int16_t pitch_loops_pid(Error_position *angle_error,Error_position *gyro_error,int16_t feedback1,int16_t feedback2);
#endif
