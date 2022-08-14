#ifndef __DRV_CAN_H__

#define __DRV_CAN_H__

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#define MOTO_ID_1  0x201 
#define MOTO_ID_2  0x202
#define MOTO_ID_3  0x203
#define MOTO_ID_4  0x204
#define MOTO_ID_5  0x205
#define MOTO_ID_6  0x206
#define MOTO_ID_7  0x207
#define MOTO_ID_8  0x208
#define MOTO_ID_9  0x209
#define MOTO_ID_10 0X20A

extern EventGroupHandle_t VerifyHandle;

#define VerifyMotor_1 0x01  
#define VerifyMotor_2 0x02 
#define VerifyMotor_3 0x04 
#define VerifyMotor_4 0x08  
#define VerifyMotor_5 0x10  
#define VerifyMotor_6 0x20  
#define VerifyMotor_7 0x40
#define VerifyMotor_8 0x80
#define VerifyMotor_9 0x100

typedef struct
{
	int16_t angle;     //����Ļ�е�Ƕ�����
	int16_t speed;     //�����ʵʱ�����ٶ�
	int16_t current;   //�����ʵ��ת�ص���
	int8_t  celsius;   //����¶�
}MotoData;

typedef struct
{
	int16_t angle;     //����Ļ�е�Ƕ�����
	int16_t last_angle; //����ϴεĻ�е�Ƕ�
	int16_t speed;     //�����ʵʱ�����ٶ�
	int16_t current;   //�����ʵ��ת�ص���
	int8_t  celsius;   //����¶�
	int16_t turns;   //��е�۵����תȦ��
	
	
}Arm_MotoData;



extern uint32_t TxMailbox;					//����ָʾCAN��Ϣ���ͺ���HAL_CAN_AddTxMessageʹ�����ĸ���������������
extern int16_t yaw_angle,pitch_angle;
extern int16_t yaw_speed,pitch_speed;

extern CAN_TxHeaderTypeDef Tx1Message;
extern CAN_RxHeaderTypeDef Rx1Message;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern MotoData Moto_1;
extern MotoData Moto_2;
extern MotoData Moto_3;
extern MotoData Moto_4;
extern Arm_MotoData Moto_5;//1��̧�����         
extern Arm_MotoData Moto_6;//2������  
extern Arm_MotoData Moto_7;//2���ҵ��   
extern Arm_MotoData Moto_8;//3��̧����� 
extern Arm_MotoData Moto_9;//4����ת���  



void CanFilter_Init(void);//CAN_HandleTypeDef *hcan
void Can_Tx1MesInit(CAN_TxHeaderTypeDef *Tx1Mes);
void Can_Rx1MesInit(CAN_RxHeaderTypeDef *Rx1Mes);
void Can_Init(void);
void Can_SendMoto_Chassis(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4);
void Can_SendMoto_Arm(int16_t Current,uint16_t ID);
void Can_SendMoto_Arm2(int16_t Current,uint16_t ID);
#endif
