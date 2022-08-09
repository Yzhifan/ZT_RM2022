#ifndef __EMERGENCY_H

#define __EMERGENCY_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void EmergencyTask(void const * argument);

#endif
