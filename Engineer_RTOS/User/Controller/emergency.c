#include "emergency.h"
#include "verify.h"
#include "buzzer.h"

extern TIM_HandleTypeDef htim12;
extern EventGroupHandle_t VerifyHandle;
extern uint8_t chasis_error[4],arm_error[5],gyro_error[2],dr16_error;
extern uint8_t chasiss_check,arm_check,gyro_check,dr16_check;

void EmergencyTask(void const * argument)
{
	
	  for(;;)
  {

		if(chasiss_check != check_ok)//底盘通讯有误
		{
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 50);  //音量
			/* 执行响一次，停半秒，代表底盘出了问题*/
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
			osDelay(500);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 15);  //音量
			/* 挨个判断哪个电机有错，有错就执行一遍该电机编号的报警程序*/
			for(uint8_t i= 0;i<4;i++)
			{
				if(chasis_error[i])
				{
					chas_buzzer(i+1);
				}
			}
			osDelay(1000);
		}
		if(arm_check != check_ok)//机械臂通讯有误
		{
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 50);  //音量
			/* 执行响两次次，停半秒，代表机械臂出了问题*/
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
			osDelay(500);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 15);  //音量
			/* 挨个判断哪个电机有错，有错就执行一遍该电机编号的报警程序*/
			for(uint8_t i= 0;i<5;i++)
			{
				if(arm_error[i])
				{
					arm_buzzer(i+1);
				}
			}
			osDelay(1000);
		}
//		#if 1
		if(gyro_check != check_ok)//陀螺仪通讯有误
		{
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 50);  //音量
			/* 执行响三次，停半秒，代表底盘出了问题*/
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
			osDelay(150);
			HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
			osDelay(500);
			__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 15);  //音量
			/* 挨个判断哪个电机有错，有错就执行一遍该电机编号的报警程序*/
			for(uint8_t i= 0;i<2;i++)
			{
				if(gyro_error[i])
				{
					gyro_buzzer(i+1);
				}
			}
			osDelay(500);
		}
//		#endif
		if(dr16_check != check_ok)//陀螺仪通讯有误
		{
			dr16_buzzer();
		}

  }

}
