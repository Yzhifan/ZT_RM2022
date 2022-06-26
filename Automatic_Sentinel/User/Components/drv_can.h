#ifndef __DRV_CAN_H
#define __DRV_CAN_H

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
	int16_t angle;     //电机的机械角度数据
	int16_t speed;     //电机的实时反馈速度
	int16_t current;   //电机的实际转矩电流
	int8_t  celsius;   //电机温度
}MotoData;

typedef struct
{
	int16_t angle;   //电机的机械角度
	int16_t last_angle;
	int16_t Torque_current;//电机的实际转矩电流
	int16_t real_current;//电机的发送电流
	int16_t speed;
}Moto6623Data;

typedef struct
{
	int16_t integral;
	int16_t temp;
	int16_t number;
}Speed_convert;

extern CAN_HandleTypeDef hcan1;

extern int16_t yaw_angle,pitch_angle;
extern int16_t yaw_speed,pitch_speed;
extern int16_t Raw_yawData,Raw_pitchData;
extern MotoData Moto_1;
extern MotoData Moto_2;
extern MotoData Moto_3;
extern MotoData Moto_4;
extern Moto6623Data Moto_5;//五号电机的数据结构体
extern Moto6623Data Moto_6;

void Can_Init(void);
void Can_SendMoto_chasiss(int16_t Current,uint16_t ID);
void Can_SendMoto_gimbals(int16_t Current,uint16_t ID);


#endif
