#include "gimbals.h"
#include "pid.h"
#include "drv_can.h"
#include "drv_usart.h"
#include "DR16.h"
#include "jy901s.h"
#include "coordinate.h"

#define Gyro_mode 1  //根据陀螺仪回传的数据控制云台
#define Motor_mode 3 //根据电机自身的机械角度控制云台

extern uint8_t arm_check,arm_error[5];


/*云台电机结构体*/
Error_position arm1_gyro_error;
Error_position arm2_gyro_error1;
Error_position arm2_gyro_error2;

/* 云台控制任务分为三个部分，第一关节和第二关节、第三关节、第四关节旋转电机
	 第一部分：由第一关节和第二关节经过两个解算函数解算出的陀螺仪角度完成机械臂伸展（x，y）的坐标
	 第二部分：由第三关节通过计算与第二关节的相对角度来保持第三关节的水平，当第一部分到达位移后，解除水平计算
						利用pid控制电机转速，人工调节第三关节的角度
	 第三部分：
*/

/**
* @brief  云台运动控制函数
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


