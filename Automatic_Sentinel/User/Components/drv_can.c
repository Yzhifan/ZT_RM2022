#include "drv_can.h"

int16_t yaw_angle,pitch_angle;//pitch 7350-8050
int16_t yaw_speed,pitch_speed;//��λ����/s
int16_t filtered_speed5,filtered_speed6;
int16_t Raw_yawData,Raw_pitchData;
float speed_a = 0.95;

uint8_t CAN_RxData[8],CAN_Tx1Data[8],CAN_Tx2Data[8]; //����������λ�޷������������Ҫ���͵����ݺͽ��յ�������
uint32_t TxMailbox = 0;					//����ָʾCAN��Ϣ���ͺ���HAL_CAN_AddTxMessageʹ�����ĸ���������������

CAN_TxHeaderTypeDef Tx1Message;  //����һ������ΪTx1Message��CAN_TxHeaderTypeDef���͵Ľṹ�壬������ŷ������ݵ�����
CAN_RxHeaderTypeDef Rx1Message;  //

MotoData Moto_1;//����1�ŵ��
MotoData Moto_2;//����2�ŵ��
MotoData Moto_3;//����3�ŵ��
MotoData Moto_4;//����4�ŵ��
Moto6623Data Moto_5;//yaw��6623
Moto6623Data Moto_6;//pitch��6623
Speed_convert yaw_moto;
Speed_convert pitch_moto;


extern int16_t angle_exp,gyro_real;

/**
* @brief  CANͨѶ�˲����������˲�Ҫ����Ϣ
* @param  None
* @return None
*/
void CanFilter_Init(void)//�����˲���ѡ�������Ҫ����Ϣ��
{
	CAN_FilterTypeDef Canfilter;  //����һ������ΪCanfilter��CAN_FilterTypeDef���͵Ľṹ��
	
	
	Canfilter.FilterMode = CAN_FILTERMODE_IDMASK;  ///����Ϊ����ģʽ
	Canfilter.FilterScale = CAN_FILTERSCALE_32BIT;  ///32λ������ģʽ����32λ������ģʽ������16λ�ģ�
	
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
* @brief  CANͨѶ��ʼ����������������can�����Ҫ�Ĺ���
* @param  None
* @return None
*/
void Can_Init(void)
{
	CanFilter_Init();

	HAL_CAN_Start(&hcan1);  //����CAN1 
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�,��FIFO0���յ�����ʱ�����ж�
	
}
	
/**
* @brief  CAN���ͺ���������IDΪ0x201-0x204�ĵ��
* @param  Ҫ���͵�����ĵ���ֵ
* @param  Ҫ���Ƶĵ��ID��
* @return None
*/
void Can_SendMoto_chasiss(int16_t Current,uint16_t ID)
{
	
	Tx1Message.StdId = 0x200; //������ͱ�ʶ��id
	Tx1Message.ExtId = 0x00;  //��������չ֡
	Tx1Message.IDE   = CAN_ID_STD;  //֡����  ��׼֡  �����ǲ��õ��Ǳ�׼֡������֮�⻹����չ֡����չ֡���ŵ���ID���������õø��࣬�������ò��ϣ�
	Tx1Message.RTR   = CAN_RTR_DATA;//֡��ʽ  ����֡   ���������Ϊ�����������ݵ�֡���ͣ������֮�⻹��ң��֡����ɶ���в�ѯ��
	Tx1Message.DLC   = 8;          //��Ҫ���͵����ݳ��ȣ�����˵��������������8���ֽ�
	Tx1Message.TransmitGlobalTime = DISABLE;  //ָ���Ƿ��ڿ�ʼʱ����ʱ���������ֵ,û��Ҳ�ò��ϣ���������Ҳ��ʹ�ܣ���֪��Ϊɶ�������Լ�ȥ�����Ǹ����õ�
	
	switch ( ID ) ///���������ID������ֵ��ֵ���ض�������λ
	{
		case 0x201:
		{
			CAN_Tx1Data[0] = (int8_t)(Current>>8);   //�����÷��������Ϊǿ��ת��+��λ���Ƚ����������16λ����ǿ��ת����8λ���ݣ���ʱ�߰�λ�ᱻ����ȥ��Ҳ������ʧ��������Ϊ���ǽ�ԭ��
			CAN_Tx1Data[1] = (int8_t)Current;        //16λ������������8λ����ԭ���ڸ����İ�λ�ƶ����˵Ͱ�λ�����磺12345678 00000000���16λ�����ݾ�����������00000000 12345678 
			                                        //����Ϊ�����Ѿ���ǿ��ת��Ϊ8λ�ģ���8λ��λ���Ѿ�ȡ���ˣ����ʱCAN_TxData[0]װ��12345678��˸�ԭ��Current�߰�λ������
		}break;
	  case 0x202:
		{
			CAN_Tx1Data[2] = (int8_t)(Current>>8);
			CAN_Tx1Data[3] = (int8_t)Current;
		}break;
		case 0x203:
		{
			CAN_Tx1Data[4] = (int8_t)(Current>>8);
			CAN_Tx1Data[5] = (int8_t)Current;
		}break;
		case 0x204:
		{
			CAN_Tx1Data[6] = (int8_t)(Current>>8);
			CAN_Tx1Data[7] = (int8_t)Current;
		}break;		
	}

	HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,CAN_Tx1Data,&TxMailbox);
}

/**
* @brief  CAN���ͺ���������IDΪ0x205-0x208�ĵ��
* @param  Ҫ���͵�����ĵ���ֵ
* @param  Ҫ���Ƶĵ��ID��
* @return None
*/
void Can_SendMoto_gimbals(int16_t Current,uint16_t ID)
{
	
	Tx1Message.StdId = 0x1FF; //������ͱ�ʶ��id
	Tx1Message.ExtId = 0x00;  //��������չ֡
	Tx1Message.IDE   = CAN_ID_STD;  //֡����  ��׼֡  �����ǲ��õ��Ǳ�׼֡������֮�⻹����չ֡����չ֡���ŵ���ID���������õø��࣬�������ò��ϣ�
	Tx1Message.RTR   = CAN_RTR_DATA;//֡��ʽ  ����֡   ���������Ϊ�����������ݵ�֡���ͣ������֮�⻹��ң��֡����ɶ���в�ѯ��
	Tx1Message.DLC   = 8;          //��Ҫ���͵����ݳ��ȣ�����˵��������������8���ֽ�
	Tx1Message.TransmitGlobalTime = DISABLE;  //ָ���Ƿ��ڿ�ʼʱ����ʱ���������ֵ,û��Ҳ�ò��ϣ���������Ҳ��ʹ�ܣ���֪��Ϊɶ�������Լ�ȥ�����Ǹ����õ�
	
	switch ( ID ) ///���������ID������ֵ��ֵ���ض�������λ
	{
		case 0x205:
		{
			CAN_Tx2Data[0] = (int8_t)(Current>>8);   //�����÷��������Ϊǿ��ת��+��λ���Ƚ����������16λ����ǿ��ת����8λ���ݣ���ʱ�߰�λ�ᱻ����ȥ��Ҳ������ʧ��������Ϊ���ǽ�ԭ��
			CAN_Tx2Data[1] = (int8_t)Current;        //16λ������������8λ����ԭ���ڸ����İ�λ�ƶ����˵Ͱ�λ�����磺12345678 00000000���16λ�����ݾ�����������00000000 12345678 
			                                        //����Ϊ�����Ѿ���ǿ��ת��Ϊ8λ�ģ���8λ��λ���Ѿ�ȡ���ˣ����ʱCAN_TxData[0]װ��12345678��˸�ԭ��Current�߰�λ������
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
				/*�����״̬�������ۼ�ֵ���ڱ��νǶ�ֵ��ȥ�ϴνǶ�ֵ*/
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
		if(moto->number == 5)
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
* @brief  CAN�����жϻص����������ղ�����������ֵ
* @param  CAN���սṹ��
* @return None
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
			case 0x204:
			{
				Moto_4.speed = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
			}break;
			case 0x205:
			{
				Moto_5.angle = ((int16_t)CAN_RxData[0]<<8|(int16_t)CAN_RxData[1]);
				Moto_5.real_current = ((int16_t)CAN_RxData[2]<<8|(int16_t)CAN_RxData[3]);
				
				/*��yaw��6623�ĽǶ�����΢�ֹ��˳��ٶ�*/
				Raw_yawData = convert_6623speed(&yaw_moto,&Moto_5); //*200/22.755f
				Moto_5.speed = (int16_t)(speed_a * Raw_yawData + (1 - speed_a)*filtered_speed5);
				yaw_speed = Moto_5.speed * 200/22.755f;
				filtered_speed5 = Moto_5.speed;
				/*�������ʼ�Ƕȹ���*/
				yaw_angle = Moto_5.angle - 6900;
				if(yaw_angle>4095)
				{
					yaw_angle -=8191;
				}
				else if(yaw_angle<-4095)
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
				pitch_speed = Moto_6.speed * 200/22.755f;
				filtered_speed6 = Moto_6.speed;
				
				pitch_angle = Moto_6.angle - 1800;
				if(pitch_angle>4095)
				{
					pitch_angle -=8191;
				}
				else if(pitch_angle<-4095)
				{
					pitch_angle +=8191;
				}

			}break;

		}
			if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///����жϹ����Ƿ�����������Ļ�����������
		{
			__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
	
	
}




