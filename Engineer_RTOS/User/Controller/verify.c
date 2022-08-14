#include "verify.h"

extern EventGroupHandle_t VerifyHandle;
extern osThreadId VerifyTaskHandle;
extern osThreadId emergencyTaskHandle;//Ӧ����������
extern osThreadId chasissTaskHandle;//�����˶�����
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
		
		vTaskResume(emergencyTaskHandle);//�ͷŽ������񣬵ȴ�У������У��
		
		/*  ң�ؽ��ջ�  ���������� ��е�۵�� �����ĵ��   1~4λΪ����У�飬5~9λΪ��е��У�飬10~11λΪ������У�飬12λΪң����У��*/
		/*      1        11        1 1111    1111*/   
		
			/* �����¼��������λ����ȡ������У������λ��1��ֵ����Ӧ������*/
		Verify_Data = ~xEventGroupWaitBits(VerifyHandle,0x0FFF,pdFALSE,pdTRUE,0);
		
		/* 12��ѭ��Ѳ������λ��,���Գ����λ��ֵ����Ӧ������*/
		for(uint8_t i= 0;i<12;i++)
		{
			error[i] = Verify_Data&temp;    //�����㣬��λ�룬��У��״̬����뵽error����
			Verify_Data = Verify_Data >> 1; //����һλ
			
			if( i<4) //����У�鸳ֵ
			{
				chasis_error[i] = error[i]; 
			}
			else if(i<9) //��е��У�鸳ֵ
			{
				arm_error[i-4] = error[i];
			}
			else if(i<11) //������У�鸳ֵ
			{
				gyro_error[i-9] = error[i];
			}
			else dr16_error = error[i];
		}
		
		/* ������̵��ͨѶ״̬*/
		if(chasis_error[0]|chasis_error[1]|chasis_error[2]|chasis_error[3])
		{
			chasiss_check = check_err;//���̵����ͨѶ�����
		}
		else chasiss_check = check_ok;//����ͨѶ����
		/* �����е�۵��ͨѶ״̬*/
		if(arm_error[0]|arm_error[1]|arm_error[2]|arm_error[3]|arm_error[4])
		{
			arm_check = check_err;
		}
		else arm_check = check_ok;
		/* ����������ͨѶ״̬*/
		if(gyro_error[0]|gyro_error[1])
		{
			gyro_check = check_err;
		}
		else gyro_check = check_ok;
		/* ����ң�ؽ��ջ�ͨѶ״̬*/
		if(dr16_error)
		{
			dr16_check = check_err;
		}
		else dr16_check = check_ok;
		
	
		test = xEventGroupWaitBits(VerifyHandle,0x0FFF,pdTRUE,pdTRUE,portMAX_DELAY);
		
		HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1);
		vTaskSuspend(emergencyTaskHandle);//У��ʧ����У�������������޷��������������ִ�н�������
  }
}

		/* ������̵��ͨѶ״̬*/
//		chasiss_check = check_err;//��ȫ�ֱ�����Ϊ���䱨�Ļ����Ƿ�����㣿
//		xEventGroupWaitBits(VerifyHandle,0x0F,pdTRUE,pdTRUE,portMAX_DELAY);
//		chasiss_check = check_ok;
//		
//		/* �����е�۵��ͨѶ״̬*/
//		arm_check = check_err;
//		xEventGroupWaitBits(VerifyHandle,0x01F0,pdTRUE,pdTRUE,portMAX_DELAY);
//		arm_check = check_ok;
//		
//		/* ����������ͨѶ״̬*/
//		gyro_check = check_err;
//		xEventGroupWaitBits(VerifyHandle,0x600,pdTRUE,pdTRUE,portMAX_DELAY);
//		gyro_check = check_ok;
//		
//		/* ����ң�ؽ��ջ�ͨѶ״̬*/
//		dr16_check = check_err;
//		xEventGroupWaitBits(VerifyHandle,0x800,pdTRUE,pdTRUE,portMAX_DELAY);
//		dr16_check = check_ok;
