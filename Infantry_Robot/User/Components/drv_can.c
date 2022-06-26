#include "drv_can.h"

int16_t yaw_angle,pitch_angle;
int16_t yaw_gyro,pitch_gyro;
int16_t filtered_speed6;
int16_t Raw_pitchData;
float speed_a = 0.9;

uint8_t CAN_Tx1Data[8],CAN_Tx2Data[8],CAN_RxData[8]; //����������λ�޷������������Ҫ���͵����ݺͽ��յ�������
uint32_t TxMailbox = 0;					//����ָʾCAN��Ϣ���ͺ���HAL_CAN_AddTxMessageʹ�����ĸ���������������

CAN_TxHeaderTypeDef Tx1Message;  //����һ������ΪTx1Message��CAN_TxHeaderTypeDef���͵Ľṹ�壬������ŷ������ݵ�����
CAN_RxHeaderTypeDef Rx1Message;  //ͬ��

MotoData Moto_1;//����1�ŵ��      
MotoData Moto_2;//����2�ŵ��       
MotoData Moto_3;//����3�ŵ��     
MotoData Moto_4;//����4�ŵ��     
MotoData Moto_5;//��̨yaw����        
Moto6623Data Moto_6;//��̨pitch���� 
MotoData Moto_7;//�������    

/*6623��̨����ĽǶ�ת�ٶ�����ṹ��*/
Speed_convert yaw_moto;
Speed_convert pitch_moto;


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
* @brief  ����CANͨѶ��ʼ������
* @param  None
* @return None
*/
void Can_Init(void)
{
	
	CanFilter_Init();
	
	HAL_CAN_Start(&hcan1);  
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�,��FIFO0���յ�����ʱ�����ж�
	
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
	
	CAN_Tx1Data[0] = (int8_t)(Current1>>8);
	CAN_Tx1Data[1] = (int8_t)Current1;
	
	CAN_Tx1Data[2] = (int8_t)(Current2>>8);
	CAN_Tx1Data[3] = (int8_t)Current2;
	
	CAN_Tx1Data[4] = (int8_t)(Current3>>8);
	CAN_Tx1Data[5] = (int8_t)Current3;
	
	CAN_Tx1Data[6] = (int8_t)(Current4>>8);
	CAN_Tx1Data[7] = (int8_t)Current4;
	
	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,CAN_Tx1Data,&TxMailbox);
}

/**
* @brief  ���������yaw�������ͺ���������ID��0x205-208
* @param  ������͵���ֵ�����ID��
* @return None
*/
void Can_SendMoto_Gimbals(int16_t Current,uint16_t ID)
{
	Tx1Message.StdId = 0x1FF; 
	Tx1Message.ExtId = 0x00;  
	Tx1Message.IDE   = CAN_ID_STD;  
	Tx1Message.RTR   = CAN_RTR_DATA;
	Tx1Message.DLC   = 8;        
	Tx1Message.TransmitGlobalTime = DISABLE; 
	
	switch ( ID ) 
	{
		case 0x205:
		{
			CAN_Tx2Data[0] = (int8_t)(Current>>8);   
			CAN_Tx2Data[1] = (int8_t)Current;       
		}break;
	  case 0x206:
		{
			CAN_Tx2Data[2] = (int8_t)(Current>>8);
			CAN_Tx2Data[3] = (int8_t)Current;
		}break;
		case 0x207:
		{
			CAN_Tx2Data[4] = (int8_t)(Current>>8);
			CAN_Tx2Data[5] = (int8_t)Current;
		}break;
		case 0x208:
		{
			CAN_Tx2Data[6] = (int8_t)(Current>>8);
			CAN_Tx2Data[7] = (int8_t)Current;
		}break;		
	}

	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,CAN_Tx2Data,&TxMailbox);
}

/**
* @brief  6623�����λ������ת���ٶȺ���
* @param  ����ٶ�ת���Ľṹ��
* @return �����η���ֵ���ۼ�
*/
int16_t convert_6623speed(Speed_convert *moto,Moto6623Data *Moto_X)
{
	
	/*���Ƕ����ݻ����ۼ�*/
		if(( Moto_X->angle - Moto_X->last_angle )< 100&&( Moto_X->angle - Moto_X->last_angle )> -100)
			{
				moto->integral += Moto_X->angle - Moto_X->last_angle;
			}
		else if(( Moto_X->angle - Moto_X->last_angle )< -100)
			{
				moto->integral += Moto_X->angle + (8192 - Moto_X->last_angle );
			}					
		else {
				moto->integral += (8192-Moto_X->angle) +  Moto_X->last_angle ;
			}
		moto->number++;
			
	/*���ۼӵĴ��������жϣ����򽫷�����ֵ���£������Է����ϴε�ֵ*/		
		if(moto->number >= 5)
			{
				moto->number = 0; 
				moto->temp = moto->integral;
				moto->integral = 0;
				Moto_X->last_angle = Moto_X->angle;
				return moto->temp;
			}
		else 
		{
			Moto_X->last_angle = Moto_X->angle;
			return moto->temp;
		}

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
			}break;
			case 0x202:
			{
				Moto_2.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
			}break;
			case 0x203:
			{
				Moto_3.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
			}break;
			case 0x204:
			{
				Moto_4.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
			}break;
			case 0x205:
			{
				Moto_5.angle = ((int16_t)CAN_RxData[0]<<8|(int16_t)CAN_RxData[1]);
				Moto_5.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				Moto_5.current = ((int16_t)CAN_RxData[4]<<8|(int16_t)CAN_RxData[5]);
				yaw_gyro = Moto_5.speed * 6;
				
				yaw_angle = Moto_5.angle - 6845;
				if(yaw_angle>4095)
				{
					yaw_angle -=8191;
				}
				if(yaw_angle<-4095)
				{
					yaw_angle +=8191;
				}
			}break;
			case 0x206:
			{
				Moto_6.angle = ((int16_t)CAN_RxData[0]<<8|(int16_t)CAN_RxData[1]);//430
				Moto_6.real_current = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				
				Raw_pitchData = convert_6623speed(&pitch_moto,&Moto_6); //*200/22.755f
				Moto_6.speed = (int16_t)(speed_a * Raw_pitchData + (1 - speed_a)*filtered_speed6);
				pitch_gyro = Moto_6.speed * 200/22.755f;
				filtered_speed6 = Moto_6.speed;
				
				
				pitch_angle = Moto_6.angle - 5800;
				if(pitch_angle>4095)
				{
					pitch_angle -=8191;
				}
				if(pitch_angle<-4095)
				{
					pitch_angle +=8191;
				}
			}break;
			case 0x207:
			{
				Moto_7.angle = ((int16_t)CAN_RxData[0]<<8|(int16_t)CAN_RxData[1]);
				Moto_7.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				Moto_7.current = ((int16_t)CAN_RxData[4]<<8|(int16_t)CAN_RxData[5]);
			}break;

		}
			if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///����жϹ����Ƿ�����������Ļ�����������
		{
			__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
	
}

	
