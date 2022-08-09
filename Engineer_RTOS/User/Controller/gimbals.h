#ifndef __GIMBALS_H
#define __GIMBALS_H

#include "main.h"
#include "stm32f4xx_hal.h"


extern int16_t angle_exp,gyro_real;

void gimbals_controlTask(void const * argument);

#endif
