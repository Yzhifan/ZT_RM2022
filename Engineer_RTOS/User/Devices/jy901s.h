#ifndef __JY901S_H
#define __JY901S_H

#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern float gyro[3],arm1_angle[3],arm2_angle[3],temperate;

void Gyro_handler(uint8_t number);

#endif
