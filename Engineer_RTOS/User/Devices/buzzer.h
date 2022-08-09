#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void chas_buzzer (uint8_t number);
void arm_buzzer(uint8_t number);
void gyro_buzzer(uint8_t number);
void dr16_buzzer(void);

#endif
