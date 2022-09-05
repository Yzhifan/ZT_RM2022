/* ���ļ�Ϊ���̻�е������������*/
#include "coordinate.h"
#include "drv_can.h"

#include "arm_math.h"

int16_t real_x,real_y;
float coordinate_data[4];
/**
* @brief  ��е�۵�һ�ؽڽ��㺯��
* @param  x��Ŀ��ֵ��y��Ŀ��ֵ��һ�Ż�е��������ʵʱ�����ǣ����Ǳ�������ϵ��
* @return ��һ�ؽڵ�����ֵ
*/
int16_t Arm_Out1(float x,float y,float angle,float p)
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
	
	if(exp_angle> 45.8f)
	{
		exp_angle = 45.8f;
	}
	else if(exp_angle < -5.0f)
	{
		exp_angle = -5.0f;
	}
	
	coordinate_data[0] = exp_angle;
	
	/*Ŀ��Ƕ� - ʵʱ�Ƕ� ��û�е�۽Ƕ����ֵ*/
	Moto_out = (int16_t)((exp_angle - angle) * p);
	
	return Moto_out;
}

/**
* @brief  ��е�۵ڶ��ؽڽ��㺯��
* @param  x��Ŀ��ֵ��y��Ŀ��ֵ�����Ż�е��������ʵʱ�����ǣ����Ǳ�������ϵ��
* @return �ڶ��ؽڵ�����ֵ
*/
int16_t Arm_Out2(float x,float y,float angle,float p)
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
	exp_angle = (arm_angle1 + acos(all2))*180.0/Pi - 180.0;
	
	coordinate_data[1] = exp_angle;
	
	coordinate_data[2] = (arm_angle1 + acos(all2))*180.0/Pi;
	Moto_out = (int16_t)((exp_angle - angle) * p);
	
	
	return Moto_out;
}
	
