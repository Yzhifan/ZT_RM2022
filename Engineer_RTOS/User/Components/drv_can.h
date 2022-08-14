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
	int16_t angle;     //电机的机械角度数据
	int16_t speed;     //电机的实时反馈速度
	int16_t current;   //电机的实际转矩电流
	int8_t  celsius;   //电机温度
}MotoData;

typedef struct
{
	int16_t angle;     //电机的机械角度数据
	int16_t last_angle; //电机上次的机械角度
	int16_t speed;     //电机的实时反馈速度
	int16_t current;   //电机的实际转矩电流
	int8_t  celsius;   //电机温度
	int16_t turns;   //机械臂电机旋转圈数
	
	
}Arm_MotoData;



extern uint32_t TxMailbox;					//用于指示CAN消息发送函数HAL_CAN_AddTxMessage使用了哪个邮箱来发送数据
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
extern Arm_MotoData Moto_5;//1号抬升电机         
extern Arm_MotoData Moto_6;//2号左电机  
extern Arm_MotoData Moto_7;//2号右电机   
extern Arm_MotoData Moto_8;//3号抬升电机 
extern Arm_MotoData Moto_9;//4号旋转电机  



void CanFilter_Init(void);//CAN_HandleTypeDef *hcan
void Can_Tx1MesInit(CAN_TxHeaderTypeDef *Tx1Mes);
void Can_Rx1MesInit(CAN_RxHeaderTypeDef *Rx1Mes);
void Can_Init(void);
void Can_SendMoto_Chassis(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4);
void Can_SendMoto_Arm(int16_t Current,uint16_t ID);
void Can_SendMoto_Arm2(int16_t Current,uint16_t ID);
#endif
