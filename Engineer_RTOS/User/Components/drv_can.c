#include "drv_can.h"

int16_t yaw_angle,pitch_angle;
int16_t yaw_speed,pitch_speed;//��λ����/s

uint8_t CAN_TxData[8],CAN_RxData[8]; //����������λ�޷������������Ҫ���͵����ݺͽ��յ�������
uint8_t CAN2_TxData[8],CAN2_TxData2[8],CAN2_RxData[8];
uint32_t TxMailbox = 0;					//����ָʾCAN��Ϣ���ͺ���HAL_CAN_AddTxMessageʹ�����ĸ���������������

CAN_TxHeaderTypeDef Tx1Message;  //����һ������ΪTx1Message��CAN_TxHeaderTypeDef���͵Ľṹ�壬������ŷ������ݵ�����
CAN_RxHeaderTypeDef Rx1Message;  //ͬ��

CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

MotoData Moto_1;//����1�ŵ��     CAN1   
MotoData Moto_2;//����2�ŵ��     CAN1    
MotoData Moto_3;//����3�ŵ��     CAN1
MotoData Moto_4;//����4�ŵ��     CAN1
Arm_MotoData Moto_5;//1��̧�����     CAN2     
Arm_MotoData Moto_6;//2������       CAN2
Arm_MotoData Moto_7;//2���ҵ��       CAN2
Arm_MotoData Moto_8;//3��̧�����      CAN2
Arm_MotoData Moto_9;//4����ת���    CAN2

BaseType_t git_test = 4;

/**
* @brief  CAN1���˲�������ѡ�������Ҫ����Ϣ��
* @param  None
* @return None
*/
void CanFilter_Init(void)
{
	CAN_FilterTypeDef Canfilter;  
	
	Canfilter.FilterMode = CAN_FILTERMODE_IDMASK;  //����Ϊ����ģʽ
	Canfilter.FilterScale = CAN_FILTERSCALE_32BIT;  //32λ������ģʽ����32λ������ģʽ������16λ�ģ�
	
	Canfilter.FilterIdHigh = 0x0200<<5;//�������Ĳο��ֲ��е�ɸѡ����Ĵ��������й涨��32λɸѡ�����Ϊ��11λΪ��׼֡ID,���Խ�ID������λ
	Canfilter.FilterIdLow  = 0x0000;    //0x200 = B10 0000 0000,   0x0200
	Canfilter.FilterMaskIdHigh = 0x03F0<<5;//������ΪB11 1110 0000 ������λ�����ID1-8��������  0x03F0
	Canfilter.FilterMaskIdLow  = 0x0000;  
	
	Canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//����FIFO0����
	Canfilter.FilterActivation = CAN_FILTER_ENABLE;//ʹ�ܹ�����
	Canfilter.FilterBank = 0;        //ʹ���ĸ���������������CAN1����ѡ0~13
	Canfilter.SlaveStartFilterBank = 14;//��can2�����������������CAN1���Բ�������Ҳ���Ա��ź����
	
	HAL_CAN_ConfigFilter(&hcan1,&Canfilter);  //�����˲�������Ĵ���������Ϊֻ�����ã�������������������������¼�뵽�Ĵ����ģ�
	                                          //��������Ϊ���������ڸĲ��������������ð�ť������ȥ�����޸ĵ�ֵ�Ż����뵽�Ĵ���
	
}
/**
* @brief  CAN2���˲�����
* @param  None
* @return None
*/
void Can2Filter_Init(void)
{
	CAN_FilterTypeDef Canfilter; 
	
	Canfilter.FilterMode = CAN_FILTERMODE_IDMASK; 
	Canfilter.FilterScale = CAN_FILTERSCALE_32BIT; 
	
	Canfilter.FilterIdHigh = 0x0200<<5;
	Canfilter.FilterIdLow  = 0x0000;   
	Canfilter.FilterMaskIdHigh = 0x0000;
	Canfilter.FilterMaskIdLow  = 0x0000;  
	
	Canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	Canfilter.FilterActivation = CAN_FILTER_ENABLE;
	Canfilter.FilterBank = 14;        
	Canfilter.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan2,&Canfilter);
}



/**
* @brief  ����CANͨѶ��ʼ������
* @param  None
* @return None
*/
void Can_Init(void)
{
	
	CanFilter_Init();
	Can2Filter_Init();
	
	HAL_CAN_Start(&hcan1);  
	HAL_CAN_Start(&hcan2);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�,��FIFO0���յ�����ʱ�����ж�
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�,��FIFO0���յ�����ʱ�����ж�
	
}
	

/**
* @brief  ���̵�����ͺ���������ID��0x201-204
* @param  1�ŵ���ĵ���ֵ��2�ŵ���ĵ���ֵ��3�ŵ���ĵ���ֵ��4�ŵ���ĵ���ֵ
* @return None
*/
///***��Ϊ�Ƕ��������̵ĵ������  һ���Զ��ĸ�����ĵ������в�����ʱЧ�Ը��ߣ��ʲ��ж�IDֱ����һ��������һ���ԶԵ����ĸ����������� ***//
///���̵��IDĬ��Ϊ 0x201-0x204����ӦCurrent1-Current4
void Can_SendMoto_Chassis (int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4)
{
	
	Tx1Message.StdId = 0x200;
	Tx1Message.ExtId = 0x00;
	Tx1Message.IDE   = CAN_ID_STD;
	Tx1Message.RTR   = CAN_RTR_DATA;
	Tx1Message.DLC   = 8;
	Tx1Message.TransmitGlobalTime = DISABLE;
	
	CAN_TxData[0] = (int8_t)(Current1>>8);
	CAN_TxData[1] = (int8_t)Current1;
	
	CAN_TxData[2] = (int8_t)(Current2>>8);
	CAN_TxData[3] = (int8_t)Current2;
	
	CAN_TxData[4] = (int8_t)(Current3>>8);
	CAN_TxData[5] = (int8_t)Current3;
	
	CAN_TxData[6] = (int8_t)(Current4>>8);
	CAN_TxData[7] = (int8_t)Current4;
	
	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,CAN_TxData,&TxMailbox);
}

/**
* @brief  ��е�۵�����ͺ���������ID��0x205-208
* @param  ������͵���ֵ�����ID��
* @return None
*/
void Can_SendMoto_Arm(int16_t Current,uint16_t ID)
{
	Tx2Message.StdId = 0x1FF; 
	Tx2Message.ExtId = 0x00;  
	Tx2Message.IDE   = CAN_ID_STD;  
	Tx2Message.RTR   = CAN_RTR_DATA;
	Tx2Message.DLC   = 8;        
	Tx2Message.TransmitGlobalTime = DISABLE; 
	
	switch ( ID ) 
	{
		case 0x205:
		{
			CAN2_TxData[0] = (int8_t)(Current>>8);   
			CAN2_TxData[1] = (int8_t)Current;       
		}break;
	  case 0x206:
		{
			CAN2_TxData[2] = (int8_t)(Current>>8);
			CAN2_TxData[3] = (int8_t)Current;
		}break;
		case 0x207:
		{
			CAN2_TxData[4] = (int8_t)(Current>>8);
			CAN2_TxData[5] = (int8_t)Current;
		}break;
		case 0x208:
		{
			CAN2_TxData[6] = (int8_t)(Current>>8);
			CAN2_TxData[7] = (int8_t)Current;
		}break;		
	}

	HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,CAN2_TxData,&TxMailbox);
}

/**
* @brief  ��е�۵�����ͺ���������ID��0x201-204
* @param  ������͵���ֵ�����ID��
* @return None
*/
void Can_SendMoto_Arm2(int16_t Current,uint16_t ID)
{
	Tx2Message.StdId = 0x200; 
	Tx2Message.ExtId = 0x00;  
	Tx2Message.IDE   = CAN_ID_STD;  
	Tx2Message.RTR   = CAN_RTR_DATA;
	Tx2Message.DLC   = 8;        
	Tx2Message.TransmitGlobalTime = DISABLE; 
	
	switch ( ID ) 
	{
		case 0x201:
		{
			CAN2_TxData2[0] = (int8_t)(Current>>8);   
			CAN2_TxData2[1] = (int8_t)Current;       
		}break;
	  case 0x202:
		{
			CAN2_TxData2[2] = (int8_t)(Current>>8);
			CAN2_TxData2[3] = (int8_t)Current;
		}break;
		case 0x203:
		{
			CAN2_TxData2[4] = (int8_t)(Current>>8);
			CAN2_TxData2[5] = (int8_t)Current;
		}break;
		case 0x204:
		{
			CAN2_TxData2[6] = (int8_t)(Current>>8);
			CAN2_TxData2[7] = (int8_t)Current;
		}break;		
	}

	HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,CAN2_TxData2,&TxMailbox);
}


/*��Ȧ����*/
int16_t Count_laps(int16_t now,int16_t last,uint16_t turns)
{
	
	if((now - last)<-3000)
	{
		turns++;
	}
	else if((now - last)>3000)
	{
		turns--;
	}
	return turns;
}

/**
* @brief  CANͨѶ�жϻص�����
* @param  CANͨѶ����ṹ��
* @return �����������
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	if(hcan == &hcan1)//�����ǲ���can1������
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx1Message,CAN_RxData);///�����жϺ��ٵ���һ�ν��պ������������Ž��գ�����ͣ�½�������
		switch(Rx1Message.StdId)  //��ѯ���յ�����Ϣ��Դ���ĸ�ID���ĸ�������ٽ����յ��ĵ�����ݷֱ��ŵ���������õ�ֵ���Թ��������ڵ���ٶ�����
		{	
			case 0x201:
			{
				
				Moto_1.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				git_test = xEventGroupSetBitsFromISR(VerifyHandle,0x01,0);
			}break;
			case 0x202:
			{
				Moto_2.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_2,0);
			}break;
			case 0x203:
			{
				Moto_3.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_3,0);
			}break;
			case 0x204:
			{
				Moto_4.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_4,0);
			}break;

		}
			if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///����жϹ����Ƿ�����������Ļ�����������
		{
			__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
	
	if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&Rx2Message,CAN2_RxData);
		
		switch(Rx2Message.StdId)  //��ѯ���յ�����Ϣ��Դ���ĸ�ID���ĸ�������ٽ����յ��ĵ�����ݷֱ��ŵ���������õ�ֵ���Թ��������ڵ���ٶ�����
		{		
			case 0x205:   //1��̧�����
			{
				Moto_5.angle = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);	//��õ����е�Ƕ�
				Moto_5.speed = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);//��õ���ٶ�
				Moto_5.current = ((int16_t)CAN2_RxData[4]<<8|(int16_t)CAN2_RxData[5]);
				
				Moto_5.turns = Count_laps(Moto_5.angle,Moto_5.last_angle,Moto_5.turns);//��������תȦ��
				Moto_5.last_angle = Moto_5.angle;//��������νǶȸ�ֵ�� ��һ�εĽǶ�
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_5,0);
			}break;
			case 0x206:   //2������
			{
				Moto_6.angle   = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_6.speed   = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				Moto_6.current = ((int16_t)CAN2_RxData[4]<<8|(int16_t)CAN2_RxData[5]);
				
				/*�������*/  
				Moto_6.turns =  Count_laps(Moto_6.angle,Moto_6.last_angle,Moto_6.turns);//��������תȦ��
				Moto_6.last_angle = Moto_6.angle;//��������νǶȸ�ֵ�� ��һ�εĽǶ�      //1:100�Ĵ����ȣ�8192*100 = 819200�ֱ���/Ȧ
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_6,0);
			}break;
			case 0x207:   //2���ҵ��
			{
				Moto_7.angle   = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_7.speed   = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				Moto_7.current = ((int16_t)CAN2_RxData[4]<<8|(int16_t)CAN2_RxData[5]);
				
				/*�������*/  
				Moto_7.turns =  Count_laps(Moto_7.angle,Moto_7.last_angle,Moto_7.turns);//��������תȦ��
				Moto_7.last_angle = Moto_7.angle;//��������νǶȸ�ֵ�� ��һ�εĽǶ�
				
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_7,0);
			}break;
			case 0x208:  //�����ؽ�̧�����
			{
				Moto_8.angle = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_8.speed = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				/*�������*/  
				Moto_8.turns =  Count_laps(Moto_8.angle,Moto_8.last_angle,Moto_8.turns);//��������תȦ��
				Moto_8.last_angle = Moto_8.angle;//��������νǶȸ�ֵ�� ��һ�εĽǶ�
				                                 //1:36�Ĵ����ȣ�8192*36  = 294912�ֱ���/Ȧ 
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_8,0);
			}break;
			case 0x204:  //��е����ת���
			{
				Moto_9.angle = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_9.speed = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				/*�������*/  
				Moto_9.turns =  Count_laps(Moto_9.angle,Moto_9.last_angle,Moto_9.turns);//��������תȦ��
				Moto_9.last_angle = Moto_9.angle;//��������νǶȸ�ֵ�� ��һ�εĽǶ�
				                                 //1:36�Ĵ����ȣ�8192*36  = 294912�ֱ���/Ȧ 
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_9,0);
			}break;
		}
				if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///����жϹ����Ƿ�����������Ļ�����������
			{
				__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
			}
	}
}

	
