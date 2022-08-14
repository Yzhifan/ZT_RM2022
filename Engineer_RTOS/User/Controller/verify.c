#include "verify.h"

extern EventGroupHandle_t VerifyHandle;
extern osThreadId VerifyTaskHandle;
extern osThreadId emergencyTaskHandle;//应急处理任务
extern osThreadId chasissTaskHandle;//底盘运动任务
extern TIM_HandleTypeDef htim12;
uint8_t chasis_error[4],arm_error[5],gyro_error[2],dr16_error;
uint8_t chasiss_check = 1,arm_check = 1,gyro_check = 1,dr16_check = 1;

EventBits_t Verify_Data,test;

void StartVerifyTask(void const * argument)
{
	uint8_t error[12];
	EventBits_t temp = 0x01;
	  for(;;)
  {
		
    osDelay(20);
		
		vTaskResume(emergencyTaskHandle);//释放紧急任务，等待校验任务校验
		
		/*  遥控接收机  两个陀螺仪 机械臂电机 底盘四电机   1~4位为底盘校验，5~9位为机械臂校验，10~11位为陀螺仪校验，12位为遥控器校验*/
		/*      1        11        1 1111    1111*/   
		
			/* 读出事件组的所有位，并取反，将校验出错的位置1赋值给对应的数组*/
		Verify_Data = ~xEventGroupWaitBits(VerifyHandle,0x0FFF,pdFALSE,pdTRUE,0);
		
		/* 12次循环巡查错误的位置,并对出错的位赋值给对应的数组*/
		for(uint8_t i= 0;i<12;i++)
		{
			error[i] = Verify_Data&temp;    //与运算，逐位与，将校验状态逐个与到error数组
			Verify_Data = Verify_Data >> 1; //右移一位
			
			if( i<4) //底盘校验赋值
			{
				chasis_error[i] = error[i]; 
			}
			else if(i<9) //机械臂校验赋值
			{
				arm_error[i-4] = error[i];
			}
			else if(i<11) //陀螺仪校验赋值
			{
				gyro_error[i-9] = error[i];
			}
			else dr16_error = error[i];
		}
		
		/* 核验底盘电机通讯状态*/
		if(chasis_error[0]|chasis_error[1]|chasis_error[2]|chasis_error[3])
		{
			chasiss_check = check_err;//底盘电机有通讯出错的
		}
		else chasiss_check = check_ok;//底盘通讯正常
		/* 核验机械臂电机通讯状态*/
		if(arm_error[0]|arm_error[1]|arm_error[2]|arm_error[3]|arm_error[4])
		{
			arm_check = check_err;
		}
		else arm_check = check_ok;
		/* 核验陀螺仪通讯状态*/
		if(gyro_error[0]|gyro_error[1])
		{
			gyro_check = check_err;
		}
		else gyro_check = check_ok;
		/* 核验遥控接收机通讯状态*/
		if(dr16_error)
		{
			dr16_check = check_err;
		}
		else dr16_check = check_ok;
		
	
		test = xEventGroupWaitBits(VerifyHandle,0x0FFF,pdTRUE,pdTRUE,portMAX_DELAY);
		
		HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
		vTaskSuspend(emergencyTaskHandle);//校验失败则校验任务阻塞，无法将紧急任务挂起，执行紧急任务
  }
}

		/* 核验底盘电机通讯状态*/
//		chasiss_check = check_err;//将全局变量改为邮箱报文机制是否更方便？
//		xEventGroupWaitBits(VerifyHandle,0x0F,pdTRUE,pdTRUE,portMAX_DELAY);
//		chasiss_check = check_ok;
//		
//		/* 核验机械臂电机通讯状态*/
//		arm_check = check_err;
//		xEventGroupWaitBits(VerifyHandle,0x01F0,pdTRUE,pdTRUE,portMAX_DELAY);
//		arm_check = check_ok;
//		
//		/* 核验陀螺仪通讯状态*/
//		gyro_check = check_err;
//		xEventGroupWaitBits(VerifyHandle,0x600,pdTRUE,pdTRUE,portMAX_DELAY);
//		gyro_check = check_ok;
//		
//		/* 核验遥控接收机通讯状态*/
//		dr16_check = check_err;
//		xEventGroupWaitBits(VerifyHandle,0x800,pdTRUE,pdTRUE,portMAX_DELAY);
//		dr16_check = check_ok;
