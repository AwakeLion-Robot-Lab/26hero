#include "bsp_can.h"
#include "detect_task.h"
#include "sys_config.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "comm_task.h"
#include "chassis_task.h"
#include "pc_tx_data.h"
#include "math.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "judge_rx_data.h"//!!!
#include "rc.h"
#include "modeswitch_task.h"

CanRxMsg rx1_message;
CanRxMsg rx2_message;

moto_measure_t moto_chassis[4];//底盘
moto_measure_t moto_pit; //pitch
moto_measure_t moto_yaw; //yaw
moto_measure_t moto_loader; //拨盘

uint16_t flag =0;
float pit_angle;
float *p_angle = &pit_angle;
float distance;
float *p_distance = &distance;

uint8_t test[8];		
uint8_t shoot_ready;			//判断是否触屏到微动开关

static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan,CanRxMsg *message)
{
	if(_hcan == CAN1)
	{
		switch(message->StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
				static uint8_t i = 0;
				//处理电机ID号
				i = message->StdId - CAN_3508_M1_ID;
				//处理电机数据宏函数
				moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], message) : encoder_data_handler(&moto_chassis[i], message);
				//WDOG
				err_detector_hook(CHASSIS_M1_OFFLINE + i);
       
			}break;
			
			case CAN_TRIGGER_MOTOR_ID:
			{
				moto_loader.msg_cnt++ <= 50 ? get_moto_offset(&moto_loader, message) : encoder_data_handler(&moto_loader, message);
				err_detector_hook(TRIGGER_MOTO_OFFLINE_REAR);  
			}break;
			
			case CAN_SUPER_CAP_ID:
			{
				chassis.CapData[0] = (float)((rx1_message.Data[1] << 8 | rx1_message.Data[0])/100.f);		//输入电压
				chassis.CapData[1] = (float)((rx1_message.Data[3] << 8 | rx1_message.Data[2])/100.f);		//电容电压
				chassis.CapData[2] = (float)((rx1_message.Data[5] << 8 | rx1_message.Data[4])/100.f);		//输入电流
				chassis.CapData[3] = (float)((rx1_message.Data[7] << 8 | rx1_message.Data[6])/100.f);		//设定功率	
			}break;
			
			default:
			{
			}break;
		}
//		if(message->StdId==0x51)//10010B
//		{
//		chassis_joint[joint_RIGHT].joint_ang_fdb= ((message-> Data[4]<<8)|(message->Data[5]))*360/32768;
//		chassis_joint[joint_RIGHT].joint_spd_fdb =((message-> Data[0]<<8)|(message->Data[1]))/10;

//    }
//		if(message->StdId==0x52)//10010B
//		{
//		chassis_joint[joint_LEFT].joint_ang_fdb= ((message-> Data[4]<<8)|(message->Data[5]))*360/32768;
//		chassis_joint[joint_LEFT].joint_spd_fdb =((message-> Data[0]<<8)|(message->Data[1]))/10;

//    }
		if(message->StdId==0xc1)//1010B
		{
		chassis_joint[joint_RIGHT].current_fdb = (message-> Data[4]<<8)|(message->Data[5]);
    BM1010b_encoder_data_handler(&chassis_joint[joint_RIGHT], message);     //先读取编码值偏移，然后正式开始计算

    }  
			if(message->StdId==0xc2)//1010B
		{
		chassis_joint[joint_LEFT].current_fdb = (message-> Data[4]<<8)|(message->Data[5]);
	  BM1010b_encoder_data_handler(&chassis_joint[joint_LEFT], message);
			
    }
	}
	else
	{
	switch(message->StdId)
    {	
			case CAN_YAW_MOTOR_ID: //接收yaw电机
			{
				encoder_data_handler(&moto_yaw, message);
				err_detector_hook(GIMBAL_YAW_OFFLINE);
			}break;
			
			case GIMBAL_MASTER_ID: //接收测距距离，pit相对角，轻触开关
			{
				uint8_t *px = rx2_message.Data;
				for(int i = 0; i < 4; i++)
				{
					*((uint8_t*)p_angle + i) = *(px + i);//pit+轻触开关
				}
				for(int i = 0; i < 4; i++)
				{
					*((uint8_t*)p_distance + i) = *(px + i + 4);//距离
				}
				pit_encode(&pit_angle); //pit百位数位轻触开关有效位，要解码得到pit和轻触开关的标志位
				gimbal.sensor.pit_relative_angle = pit_angle;
				
			}break;
			
			case GIMBAL_YAW_MASTER_ID:
			{
				uint8_t *px2 = rx2_message.Data;
				float yaw_total;
				float *p_yaw_total = &yaw_total;
				float yaw_offset;
				float *p_yaw_offset = &yaw_offset;
				for(int i = 0; i < 4; i++)
				{
					*((uint8_t*)p_yaw_total + i) = *(px2 + i);//total
				}
				for(int i = 0; i < 4; i++)
				{
					*((uint8_t*)p_yaw_offset + i) = *(px2 + i + 4);//offset
				}
				gimbal.sensor.yaw_total_angle = yaw_total;
				gimbal.yaw_offset_angle = yaw_offset;
				
			}break;
			
			case gimbal_to_chassis_rc_ID:
			{
				rc.image.sw	 	= (rx2_message.Data[0]&0x03);			//接收新遥控挡位切换
				rc.image.key_r	= ((rx2_message.Data[0]&0x07)>>2);		//接收新遥控右自定义健
				rc.image.key_l	= ((rx2_message.Data[0]&0x0F)>>3);		//接收新遥控左自定义健
				rc.image.pause	= ((rx2_message.Data[0]&0x1F)>>4);		//接收新遥控暂停健
				rc.kb.bit.SHIFT = ((rx2_message.Data[0]&0x20)>>5);		//接收键盘值
				rc.kb.bit.CTRL  = ((rx2_message.Data[0]&0x40)>>6);		
				rc.kb.bit.Z = ((rx2_message.Data[0]&0x80)>>7);
				
				rc.sw1      = (rx2_message.Data[1]&0x03);				//接收旧遥控左拨杆值
				rc.sw2      = ((rx2_message.Data[1]&0x0C)>>2);			//接收旧遥控右拨杆值
				rc.kb.bit.W = ((rx2_message.Data[1]&0x10)>>4);
				rc.kb.bit.A = ((rx2_message.Data[1]&0x20)>>5);
				rc.kb.bit.S = ((rx2_message.Data[1]&0x40)>>6);
				rc.kb.bit.D = ((rx2_message.Data[1]&0x80)>>7);
				
				
				shoot.fric_trun_on = (rx2_message.Data[2]&0x01);			//接收水平摩擦轮是否开启标志位
				gimbal.dodge_ctrl  = ((rx2_message.Data[2]&0x02)>>1);
				gimbal.separt_ctrl = ((rx2_message.Data[2]&0x04)>>2);
				global_mode = (global_status)((rx2_message.Data[2]&0x18)>>3);
				gimbal_turn_flag = (rx2_message.Data[2]&0x20)>>5;
				shoot.loader_fire_allow = ((rx2_message.Data[2]&0x40)>>6); 	//拨盘发射标志位
				rc.remote_flag = (rx2_message.Data[2]&0x80)>>7;			 	//使用新老遥控标志位
				
				rc.ch1 = ((rx2_message.Data[3]<<8)|rx2_message.Data[4]);
				rc.ch2 = ((rx2_message.Data[5]<<8)|rx2_message.Data[6]);
				rc.iw_flag = rx2_message.Data[7];							//拨轮标志位
				
				err_detector_hook(REMOTE_CTRL_OFFLINE);
			}break;
			
			case gimbal_to_chassis_yaw_ID:
			{
				uint8_t *px3 = rx2_message.Data;
				float yaw_gyro_angle;
				float *p_yaw_angle = &yaw_gyro_angle;
				for(int i = 0; i < 4; i++)
				{
					*((uint8_t*)p_yaw_angle + i) = *(px3 + i);
				}
				gimbal.sensor.yaw_gyro_angle = yaw_gyro_angle;
			}break;
			
			default:
			{
			}break;
    }
	}
}
void BM1010b_encoder_data_handler(chassis_joint_t* chassis_joint,CanRxMsg *message)
{
    uint16_t speed=0;
	
    speed=(uint16_t)(message->Data[2]<<8 | message->Data[3]);

    if(speed>32768)
       chassis_joint->joint_spd_fdb = (speed-65536);
    else
       chassis_joint->joint_spd_fdb =speed;
		
	
       chassis_joint->joint_spd_fdb = chassis_joint->joint_spd_fdb*360.0f/600.0f;   //600 = 60*10 60为60秒，10为分辨率，最终单位转化为了每秒多少度
		
//		if(ID == 0xc1)
//		{
//			joint_ang_fdb_before[joint_RIGHT] = chassis_joint->joint_ang_fdb;
//			
//		}
//		else if(ID == 0xc2)
//		{
//			joint_ang_fdb_before[joint_LEFT] = chassis_joint->joint_ang_fdb;
//		}
		
	    chassis_joint->joint_ang_fdb = (uint16_t)(message->Data[0] << 8 | message->Data[1])*360/32768;
		
//		if(ID == 0xc1)
//		{
//			if(joint_ang_fdb_before[joint_RIGHT] > 0 && joint_ang_fdb_before[joint_RIGHT] < 90 && chassis_joint->joint_ang_fdb >270 && chassis_joint->joint_ang_fdb < 360)
//			{
//				chassis_joint[joint_RIGHT].joint_ang_ref += 360;
//			}
//			if(joint_ang_fdb_before[joint_RIGHT] > 270 && joint_ang_fdb_before[joint_RIGHT] < 360 && chassis_joint->joint_ang_fdb >0 && chassis_joint->joint_ang_fdb < 90)
//			{
//				chassis_joint[joint_RIGHT].joint_ang_ref -= 360;
//			}
//			
//		}
//		else if(ID == 0xc2)
//		{
//			joint_ang_fdb_before[joint_LEFT] = chassis_joint->joint_ang_fdb;
//		}
			
			
			
//		if (ptr->ecd - ptr->last_ecd > 16384)
//	{
//			 ptr->round_cnt--;
//			 ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 32768;
//	}
//		else if (ptr->ecd - ptr->last_ecd < -16384)
//	{
//			 ptr->round_cnt++;
//			 ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 32768;
//	}
//		else
//	{
//			 ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
//	}
//			 ptr->total_ecd = ptr->round_cnt * 32768 + ptr->ecd - ptr->offset_ecd;

}


void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message)//解算
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
  
  if (ptr->ecd - ptr->last_ecd > 4096)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	
	ptr->speed_rpm     = (int16_t)(message->Data[2] << 8 | message->Data[3]);
  ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]);

}



/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}

void get_BM_Moto_offset(moto_measure_t* ptr, CanRxMsg *message)
{
    ptr->ecd        = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
    ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief  send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param  current value corresponding motor(yaw/pitch/trigger)
  */

//发送底盘电流
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = iq1 >> 8;
    TxMessage.Data[1] = iq1;
    TxMessage.Data[2] = iq2 >> 8;
    TxMessage.Data[3] = iq2;
    TxMessage.Data[4] = iq3 >> 8;
    TxMessage.Data[5] = iq3;
    TxMessage.Data[6] = iq4 >> 8;
    TxMessage.Data[7] = iq4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}

void Motor10010B_Enable(void)
{
	CanTxMsg TxMessage;
	TxMessage.DLC = 0x08;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.StdId = 0x38;//设置ID为1
	TxMessage.Data[0] = 0x02;
	TxMessage.Data[1] = 0x02;
	TxMessage.Data[2] = 0x02;
	TxMessage.Data[3] = 0x02;
	TxMessage.Data[4] = 0x02;
	TxMessage.Data[5] = 0x02;
	TxMessage.Data[6] = 0x02;
	TxMessage.Data[7] = 0x02;
	CAN_Transmit(CAN1, &TxMessage);
//while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
//while(CAN_TransmitStatus(CAN1,0) == CAN_TxStatus_Failed);

}

void Motor10010B_Current(int16_t Current1 ,int16_t Current2,int16_t Current3,int16_t Current4)
{
	CanTxMsg TxMessage;
	TxMessage.DLC = 0x08;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.StdId = 0X32;//设置ID为1
	if(Current1>7500)
	{
	Current1=7500;
	}
	if(Current1<-7500)
	{
	Current1=-7500;
	}
	if(Current2>7500)
	{
	Current2=7500;
	}
	if(Current2<-7500)
	{
	Current2=-7500;
	}
	TxMessage.Data[0] = (Current1&0xFF00)>>8;
	TxMessage.Data[1] = Current1&0x00FF;
	TxMessage.Data[2] = (Current2&0xFF00)>>8;
	TxMessage.Data[3] = Current2&0x00FF;
	TxMessage.Data[4] = (Current3&0xFF00)>>8;
	TxMessage.Data[5] = Current3&0x00FF;
	TxMessage.Data[6] = (Current4&0xFF00)>>8;
	TxMessage.Data[7] = Current4&0x00FF;
CAN_Transmit(CAN1, &TxMessage);

}

void Motor10010B_Changemode(uint8_t ID,uint8_t Motor_Mode)//要在电机失能时使用
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.DLC = 0x08;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.StdId = 0x36;
	
	
	
	switch (Motor_Mode)
	{
		case Mode_Current:
	TxMessage.Data[0] = ID;
	TxMessage.Data[1] = 0x1C;
	TxMessage.Data[2] = 0x02;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	break;
		case Mode_Speed:
	TxMessage.Data[0] = ID;
	TxMessage.Data[1] = 0x1C;
	TxMessage.Data[2] = 0x03;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	break;
		case Mode_Position:
	TxMessage.Data[0] = ID;
	TxMessage.Data[1] = 0x1C;
	TxMessage.Data[2] = 0x04;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	break;
	}
	CAN_Transmit(CAN1, &TxMessage);
//while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
//while(CAN_TransmitStatus(CAN1,0) == CAN_TxStatus_Failed);
}

void Motor10010B_FeedBackmode(uint8_t ID,uint8_t Interval,uint8_t Data0,uint8_t Data1,uint8_t Data2,uint8_t Data3)//Interval为反馈时间时间间隔(1-255ms)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.DLC = 0x08;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.StdId = 0x34;
	TxMessage.Data[0] = ID;//电机ID为1
	TxMessage.Data[1] = 0x01;//主动反馈模式
	TxMessage.Data[2] = Interval;
	TxMessage.Data[3] = Data0;
	TxMessage.Data[4] = Data1;
	TxMessage.Data[5] = Data2;
	TxMessage.Data[6] = Data3;
	TxMessage.Data[7] = 0x00;
CAN_Transmit(CAN1, &TxMessage);
//while(CAN_Transmit(CAN1, &TxMessage)==CAN_TxStatus_NoMailBox);
//while(CAN_TransmitStatus(CAN1,0) == CAN_TxStatus_Failed);
}

void Motor_Init(void)
{
//	Motor10010B_Changemode(0X01,Mode_Current);
//	Motor10010B_Changemode(0X02,Mode_Current);
	Motor10010B_Enable();
	Motor10010B_FeedBackmode(1,10,AbsolutePosition,Speed_Data,QValue,FaultInfo);
	Motor10010B_FeedBackmode(2,10,AbsolutePosition,Speed_Data,QValue,FaultInfo);
}
//发送拨盘电流
void send_trig_cur1(int16_t iq8)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_TRIG_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
	TxMessage.Data[6] = iq8 >> 8; //ID 8
    TxMessage.Data[7] = iq8;

    CAN_Transmit(LOADER_CAN1, &TxMessage);	
}

//发给功率控制板限制功率
void send_cap_power_can(uint16_t tempower)
{
	  CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CAP_POWER_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = tempower >> 8;
    TxMessage.Data[1] = tempower;

    CAN_Transmit(SUPER_CAP_CAN, &TxMessage);	
	
}

/**
函数名：send_judge_to_gimbal
作用：发送机器人ID和弹丸速度给云台开发板
**/
void send_judge_to_gimbal(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_JUDGE_TO_GIMBAL_ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = judge_recv_mesg.game_robot_state.robot_id;
	TxMessage.Data[1] = judge_recv_mesg.shoot_data.initial_speed*10;
	TxMessage.Data[2] = judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value >> 8; //发射机构每秒的冷却值
	TxMessage.Data[3] = judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value; 
	TxMessage.Data[4] = judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit >> 8;	   //发射结机构热量上限
	TxMessage.Data[5] = judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit;
	TxMessage.Data[6] = judge_recv_mesg.game_state.game_type;
	CAN_Transmit(SERVANT_CAN, &TxMessage);//CAN2
}

void pit_encode(float *angle_flag)//读取云台pit解码 百位数为轻触开关有效位，十位个位小数为pit角度，正负号不变
{
	if(*angle_flag >= 0) //100.0  000.0
	{
		if(*angle_flag - 100 >= 0)	shoot_ready = 1;
		else shoot_ready = 0;
		if(shoot_ready == 1) *angle_flag -= 100;
	}
	else     //-100.0  -000.0
	{
		if(*angle_flag + 100 <= 0)	shoot_ready = 1;
		else shoot_ready = 0;
		if(shoot_ready == 1)*angle_flag += 100;
	}
}

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        STD_CAN_RxCpltCallback(CAN1,&rx1_message);
    }
}

//can2中断
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        STD_CAN_RxCpltCallback(CAN2,&rx2_message);
    }
}


