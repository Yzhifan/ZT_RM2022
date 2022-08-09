#ifndef __DR16_H
#define __DR16_H

#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern uint8_t controller_data[36];

typedef	struct
	{ 

	  int16_t          x;
		int16_t          y;
		int16_t          z;
		int16_t      pitch;
		int16_t          f;
		
		unsigned char s1;
		unsigned char s2;
	}RC_control;
	
typedef	struct 
	{
		int16_t x;
		int16_t y;
		int16_t z;
		unsigned char L;
		unsigned char R;
	}Mouse_control;

typedef struct
{
		float x;
		float y;
		float z;
}Mouse_shift;	
	
typedef	struct
	{
		unsigned short Board; //无符号短整形
		unsigned short W;
		unsigned short S;
		unsigned short A;
		unsigned short D;
		unsigned short SHIFT;
		unsigned short CTRL;
		unsigned short Q;
		unsigned short E;
		unsigned short R;
		unsigned short F;
		unsigned short G;
		unsigned short Z;
		unsigned short X;
		unsigned short C;
		unsigned short V;
		unsigned short B;
		
	}Key_control;


extern  RC_control     RC;   							//声明遥控器数据结构体
extern Mouse_control  Mouse;
extern Key_control    Key;
extern Mouse_shift  Mouse_move;
extern int16_t Mouse_move_gyro_y;	
	
void Controller_handler(void);

#endif
