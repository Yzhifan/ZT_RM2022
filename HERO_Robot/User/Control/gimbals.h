#ifndef __GIMBALS_H
#define __GIMBALS_H

#include "main.h"
#include "stm32f4xx_hal.h"


extern int16_t angle_exp,gyro_real;

void yaw_test(void);
void gimbals_control(void);
void gimbals_cruise(void);
#endif
