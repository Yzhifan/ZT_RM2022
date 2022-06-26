#include "drv_can.h"

int16_t yaw_angle,pitch_angle;
int16_t yaw_gyro,pitch_gyro;
int16_t filtered_speed6;
int16_t Raw_pitchData;
float speed_a = 0.9;

uint8_t CAN_Tx1Data[8],CAN_Tx2Data[8],CAN_RxData[8]; //定义两个八位无符号数组来存放要发送的数据和接收到的数据
uint32_t TxMailbox = 0;					//用于指示CAN消息发送函数HAL_CAN_AddTxMessage使用了哪个邮箱来发送数据

CAN_TxHeaderTypeDef Tx1Message;  //定义一个名字为Tx1Message的CAN_TxHeaderTypeDef类型的结构体，用来存放发送数据的配置
CAN_RxHeaderTypeDef Rx1Message;  //同上

MotoData Moto_1;//底盘1号电机      
MotoData Moto_2;//底盘2号电机       
MotoData Moto_3;//底盘3号电机     
MotoData Moto_4;//底盘4号电机     
MotoData Moto_5;//云台yaw轴电机        
Moto6623Data Moto_6;//云台pitch轴电机 
MotoData Moto_7;//拨弹电机    

/*6623云台电机的角度转速度运算结构体*/
Speed_convert yaw_moto;
Speed_convert pitch_moto;


/**
* @brief  CAN1的滤波函数（选择接收想要的信息）
* @param  None
* @return None
*/
void CanFilter_Init(void)
{
	CAN_FilterTypeDef Canfilter;  
	
	Canfilter.FilterMode = CAN_FILTERMODE_IDMASK;  //设置为掩码模式
	Canfilter.FilterScale = CAN_FILTERSCALE_32BIT;  //32位的掩码模式（除32位的掩码模式，还有16位的）
	
	Canfilter.FilterIdHigh = 0x0200<<5;//根据中文参考手册中的筛选器组寄存器构成中规定，32位筛选器组成为高11位为标准帧ID,所以将ID左移五位
	Canfilter.FilterIdLow  = 0x0000;    //0x200 = B10 0000 0000,   0x0200
	Canfilter.FilterMaskIdHigh = 0x03F0<<5;//设掩码为B11 1110 0000 留低五位供电机ID1-8任意配置  0x03F0
	Canfilter.FilterMaskIdLow  = 0x0000;  
	
	Canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;//过滤FIFO0邮箱
	Canfilter.FilterActivation = CAN_FILTER_ENABLE;//使能过滤器
	Canfilter.FilterBank = 0;        //使能哪个过滤器，单独用CAN1可以选0~13
	Canfilter.SlaveStartFilterBank = 14;//给can2分配过滤器，单独用CAN1可以不鸟它，也可以备着后边用
	
	HAL_CAN_ConfigFilter(&hcan1,&Canfilter);  //开启滤波，上面的代码可以理解为只是配置，这个函数是用来将上面的设置录入到寄存器的，
	                                          //你可以理解为上面是你在改参数，这里是设置按钮，点下去了你修改的值才会输入到寄存器
	
}

/**
* @brief  开启CAN通讯初始化函数
* @param  None
* @return None
*/
void Can_Init(void)
{
	
	CanFilter_Init();
	
	HAL_CAN_Start(&hcan1);  
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断,当FIFO0接收到数据时进入中断
	
}
	

/**
* @brief  底盘电机发送函数，控制ID在0x201-204
* @param  1号电机的电流值、2号电机的电流值、3号电机的电流值、4号电机的电流值
* @return None
*/
///***因为是对整个底盘的电流输出  一次性对四个电机的电流进行操作，时效性更高，故不判断ID直接用一个函数，一次性对底盘四个电机进行输出 ***//
///底盘电机ID默认为 0x201-0x204，对应Current1-Current4
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
* @brief  拨弹电机和yaw轴电机发送函数，控制ID在0x205-208
* @param  电机发送电流值、电机ID号
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
* @brief  6623电机的位置数据转换速度函数
* @param  电机速度转换的结构体
* @return 电调五次返回值的累加
*/
int16_t convert_6623speed(Speed_convert *moto,Moto6623Data *Moto_X)
{
	
	/*做角度数据积分累加*/
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
			
	/*对累加的次数进行判断，是则将发送数值更新，否则仍发送上次的值*/		
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
* @brief  CAN通讯中断回调函数
* @param  CAN通讯处理结构体
* @return 电机各项数据
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	if(hcan == &hcan1)//看看是不是can1的数据
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx1Message,CAN_RxData);///进入中断后再调用一次接收函数，让它接着接收，不会停下接收任务
		switch(Rx1Message.StdId)  //查询接收到的消息来源于哪个ID（哪个电机）再将接收到的电机数据分别存放到各个定义好的值，以供后续调节电机速度所用
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
			if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///检查中断功能是否正常，出错的话再重新启动
		{
			__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
	
}

	
