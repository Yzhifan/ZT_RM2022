#ifndef __COORDINATE_H
#define __COORDINATE_H

#include "main.h"


#define Pi 3.1416f

#define L1  33.8f
#define L1f 1142.44f
#define L2  33.1f
#define L2f 1095.61f
#define L3  31.7f

#define initial_x 36.8f

extern int16_t coordinate_x,coordinate_y;
extern int16_t real_x,real_y;

int16_t Arm_Out1(int16_t x,int16_t y,float angle,float p);
int16_t Arm_Out2(int16_t x,int16_t y,float angle,float p);

#endif
