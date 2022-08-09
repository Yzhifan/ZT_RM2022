#ifndef __VERIFY_H
#define __VERIFY_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


#define check_ok 1
#define check_err 0

extern uint8_t chasiss_check,arm_check,gyro_check;

void StartVerifyTask(void const * argument);

#endif
