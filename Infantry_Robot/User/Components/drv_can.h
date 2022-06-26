#ifndef __DRV_CAN_H__

#define __DRV_CAN_H__

#include "main.h"
#include "stm32f4xx_hal.h"

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

typedef struct
{
	int16_t angle;     //����Ļ�е�Ƕ�����
	int16_t speed;     //�����ʵʱ�����ٶ�
	int16_t current;   //�����ʵ��ת�ص���
	int8_t  celsius;   //����¶�
}MotoData;

typedef struct
{
	int16_t angle;   //����Ļ�е�Ƕ�
	int16_t last_angle;
	int16_t Torque_current;//�����ʵ��ת�ص���
	int16_t real_current;//����ķ��͵���
	int16_t speed;
}Moto6623Data;

typedef struct
{
	int16_t integral;
	int16_t temp;
	int16_t number;
}Speed_convert;


extern uint32_t TxMailbox;					//����ָʾCAN��Ϣ���ͺ���HAL_CAN_AddTxMessageʹ�����ĸ���������������
extern int16_t yaw_angle,pitch_angle;
extern int16_t yaw_gyro,pitch_gyro;

extern CAN_TxHeaderTypeDef Tx1Message;
extern CAN_RxHeaderTypeDef Rx1Message;
extern CAN_HandleTypeDef hcan1;

extern MotoData Moto_1;//����1�ŵ��      
extern MotoData Moto_2;//����2�ŵ��       
extern MotoData Moto_3;//����3�ŵ��     
extern MotoData Moto_4;//����4�ŵ��     
extern MotoData Moto_5;//��̨yaw����        
extern Moto6623Data Moto_6;//��̨pitch���� 
extern MotoData Moto_7;//�������    


void CanFilter_Init(void);//CAN_HandleTypeDef *hcan
void Can_Tx1MesInit(CAN_TxHeaderTypeDef *Tx1Mes);
void Can_Rx1MesInit(CAN_RxHeaderTypeDef *Rx1Mes);
void Can_Init(void);
void Can_SendMoto_Chassis(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4);
void Can_SendMoto_Gimbals(int16_t Current,uint16_t ID);
#endif
