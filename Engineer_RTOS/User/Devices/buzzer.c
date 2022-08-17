#include "buzzer.h"

extern TIM_HandleTypeDef htim12;

void chas_buzzer (uint8_t number)
{
	for(uint8_t i=0;i<number; i++)
	{
		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
		osDelay(120);
		HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
		osDelay(100);
	}
		osDelay(600);
}

void arm_buzzer(uint8_t number)
{
	for(uint8_t i=0;i<number; i++)
	{
		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
		osDelay(120);
		HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
		osDelay(100);
	}
		osDelay(600);
}

void gyro_buzzer(uint8_t number)
{
	for(uint8_t i=0;i<number; i++)
	{
		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
		osDelay(120);
		HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
		osDelay(100);
	}
		osDelay(600);
}

void dr16_buzzer(void)
{
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 50);
		HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
		osDelay(150);
		HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
		osDelay(150);
}
	
