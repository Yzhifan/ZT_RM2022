#include "drv_can.h"

int16_t yaw_angle,pitch_angle;
int16_t yaw_speed,pitch_speed;//单位：°/s

uint8_t CAN_TxData[8],CAN_RxData[8]; //定义两个八位无符号数组来存放要发送的数据和接收到的数据
uint8_t CAN2_TxData[8],CAN2_TxData2[8],CAN2_RxData[8];
uint32_t TxMailbox = 0;					//用于指示CAN消息发送函数HAL_CAN_AddTxMessage使用了哪个邮箱来发送数据

CAN_TxHeaderTypeDef Tx1Message;  //定义一个名字为Tx1Message的CAN_TxHeaderTypeDef类型的结构体，用来存放发送数据的配置
CAN_RxHeaderTypeDef Rx1Message;  //同上

CAN_TxHeaderTypeDef Tx2Message;
CAN_RxHeaderTypeDef Rx2Message;

MotoData Moto_1;//底盘1号电机     CAN1   
MotoData Moto_2;//底盘2号电机     CAN1    
MotoData Moto_3;//底盘3号电机     CAN1
MotoData Moto_4;//底盘4号电机     CAN1
Arm_MotoData Moto_5;//1号抬升电机     CAN2     
Arm_MotoData Moto_6;//2号左电机       CAN2
Arm_MotoData Moto_7;//2号右电机       CAN2
Arm_MotoData Moto_8;//3号抬升电机      CAN2
Arm_MotoData Moto_9;//4号旋转电机    CAN2

BaseType_t git_test = 4;

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
* @brief  CAN2的滤波函数
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
* @brief  开启CAN通讯初始化函数
* @param  None
* @return None
*/
void Can_Init(void)
{
	
	CanFilter_Init();
	Can2Filter_Init();
	
	HAL_CAN_Start(&hcan1);  
	HAL_CAN_Start(&hcan2);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断,当FIFO0接收到数据时进入中断
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断,当FIFO0接收到数据时进入中断
	
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
* @brief  机械臂电机发送函数，控制ID在0x205-208
* @param  电机发送电流值、电机ID号
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
* @brief  机械臂电机发送函数，控制ID在0x201-204
* @param  电机发送电流值、电机ID号
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


/*计圈函数*/
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
			if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///检查中断功能是否正常，出错的话再重新启动
		{
			__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		}
	}
	
	if(hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&Rx2Message,CAN2_RxData);
		
		switch(Rx2Message.StdId)  //查询接收到的消息来源于哪个ID（哪个电机）再将接收到的电机数据分别存放到各个定义好的值，以供后续调节电机速度所用
		{		
			case 0x205:   //1号抬升电机
			{
				Moto_5.angle = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);	//获得电机机械角度
				Moto_5.speed = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);//获得电机速度
				Moto_5.current = ((int16_t)CAN2_RxData[4]<<8|(int16_t)CAN2_RxData[5]);
				
				Moto_5.turns = Count_laps(Moto_5.angle,Moto_5.last_angle,Moto_5.turns);//计算电机旋转圈数
				Moto_5.last_angle = Moto_5.angle;//将电机本次角度赋值给 上一次的角度
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_5,0);
			}break;
			case 0x206:   //2号左电机
			{
				Moto_6.angle   = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_6.speed   = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				Moto_6.current = ((int16_t)CAN2_RxData[4]<<8|(int16_t)CAN2_RxData[5]);
				
				/*间接数据*/  
				Moto_6.turns =  Count_laps(Moto_6.angle,Moto_6.last_angle,Moto_6.turns);//计算电机旋转圈数
				Moto_6.last_angle = Moto_6.angle;//将电机本次角度赋值给 上一次的角度      //1:100的传动比，8192*100 = 819200分辨率/圈
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_6,0);
			}break;
			case 0x207:   //2号右电机
			{
				Moto_7.angle   = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_7.speed   = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				Moto_7.current = ((int16_t)CAN2_RxData[4]<<8|(int16_t)CAN2_RxData[5]);
				
				/*间接数据*/  
				Moto_7.turns =  Count_laps(Moto_7.angle,Moto_7.last_angle,Moto_7.turns);//计算电机旋转圈数
				Moto_7.last_angle = Moto_7.angle;//将电机本次角度赋值给 上一次的角度
				
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_7,0);
			}break;
			case 0x208:  //第三关节抬升电机
			{
				Moto_8.angle = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_8.speed = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				/*间接数据*/  
				Moto_8.turns =  Count_laps(Moto_8.angle,Moto_8.last_angle,Moto_8.turns);//计算电机旋转圈数
				Moto_8.last_angle = Moto_8.angle;//将电机本次角度赋值给 上一次的角度
				                                 //1:36的传动比，8192*36  = 294912分辨率/圈 
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_8,0);
			}break;
			case 0x204:  //机械臂旋转电机
			{
				Moto_9.angle = ((int16_t)CAN2_RxData[0]<<8|(int16_t)CAN2_RxData[1]);
				Moto_9.speed = ((int16_t)CAN2_RxData[2]<<8|(int16_t)CAN2_RxData[3]);
				/*间接数据*/  
				Moto_9.turns =  Count_laps(Moto_9.angle,Moto_9.last_angle,Moto_9.turns);//计算电机旋转圈数
				Moto_9.last_angle = Moto_9.angle;//将电机本次角度赋值给 上一次的角度
				                                 //1:36的传动比，8192*36  = 294912分辨率/圈 
				xEventGroupSetBitsFromISR(VerifyHandle,VerifyMotor_9,0);
			}break;
		}
				if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)///检查中断功能是否正常，出错的话再重新启动
			{
				__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
			}
	}
}

	
