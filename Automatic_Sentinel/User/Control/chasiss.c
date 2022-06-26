#include "chasiss.h"
#include "drv_can.h"
#include "PID.h"
#include "DR16.h"

int16_t chas_exp;

/**
* @brief  底盘运动控制函数
* @param  None
* @return None
*/
void chasiss_control(void)
{
	int16_t chas_out;
	
	chas_out = Increment_PID(&chas1_error,Moto_3.speed,RC.x*1.5,12,1.2,0);
	
	Can_SendMoto_chasiss(chas_out,MOTO_ID_3);
}
