/* ���ļ�Ϊ���̻�е������������*/
#include "coordinate.h"
#include "drv_can.h"
#include <math.h>


int16_t coordinate_x,coordinate_y;
int16_t real_x,real_y;


/**
* @brief  ��е�۵�һ�ؽڽ��㺯��
* @param  x��Ŀ��ֵ��y��Ŀ��ֵ��һ�Ż�е��������ʵʱ�����ǣ����Ǳ�������ϵ��
* @return ��һ�ؽڵ�����ֵ
*/
int16_t Arm_Out1(int16_t x,int16_t y,float angle,float p)
{
	int16_t Moto_out,Le;//������ֵ��Ŀ���������ԭ��ֱ�߾���
	float Lf,all,exp_angle;
	
	/*������������*/
	Lf = x * x + y * y;

	Le = sqrt(Lf);
;
	
	/*�������Һ�������ֵ�ȼ����*/
	all = (L1f + Lf - L2f)/(2 * L1 * Le);

	
	/*�����һ�ؽڻ�е�۵�Ŀ��Ƕȣ�������������Ļ���ת��Ϊ�Ƕ�*/
	exp_angle = (acos(all) + atan2(y,x))*180.0/Pi;
	
	/*Ŀ��Ƕ� - ʵʱ�Ƕ� ��û�е�۽Ƕ����ֵ*/
	Moto_out = (int16_t)((exp_angle - angle) * p);
	
	return Moto_out;
}

/**
* @brief  ��е�۵ڶ��ؽڽ��㺯��
* @param  x��Ŀ��ֵ��y��Ŀ��ֵ�����Ż�е��������ʵʱ�����ǣ����Ǳ�������ϵ��
* @return �ڶ��ؽڵ�����ֵ
*/
int16_t Arm_Out2(int16_t x,int16_t y,float angle,float p)
{
	int16_t Moto_out,Le;//������ֵ��Ŀ���������ԭ��ֱ�߾���
	float Lf,all1,all2,arm_angle1,exp_angle;
	
	/*������������*/
	Lf = x * x + y * y;
	
	Le = sqrt(Lf);
	
	/*�������Һ�������ֵ�ȼ����*/
	all1 = (L1f + Lf - L2f)/(2 * L1 * Le);
	
	all2 = (L1f + L2f - Lf)/(2 * L1 * L2);
	
	arm_angle1 = acos(all1) + atan2(y,x);

	/*����ڶ��ؽڻ�е�۵�Ŀ��Ƕȣ�������������Ļ���ת��Ϊ�Ƕ�*/
	exp_angle = (arm_angle1 + acos(all2))*180.0/Pi - 180;
	
	Moto_out = (int16_t)((exp_angle - angle) * p);
	
	
	return Moto_out;
}
	
