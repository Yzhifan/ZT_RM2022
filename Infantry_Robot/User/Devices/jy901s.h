#ifndef __JY901S_H
#define __JY901S_H

#include "main.h"
#include "stm32f4xx_hal.h"

extern uint8_t Gyro_Data[22];
extern float accel[3],gyro[3],angle[3],temperate;
extern int16_t yaw_gyro_angle,pitch_gyro_angle;

void Gyro_handler(void);

#endif
