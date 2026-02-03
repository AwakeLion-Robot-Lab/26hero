#include "bsp_can.h"
#include "detect_task.h"
#include "remote_ctrl.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "comm_task.h"
#include "rc.h"
#include "shoot_task.h"
#include "pc_tx_data.h"
#include "judge_rx_data.h"
#include "keyboard.h"

// CAN消息配置宏
#define CAN_STD_DATA_FRAME(id,dlc) {      \
    .StdId = (id),                        \
    .IDE = CAN_ID_STD,                    \
    .RTR = CAN_RTR_DATA,                  \
    .DLC = (dlc)                          \
}

CanRxMsg rx1_message;
CanRxMsg rx2_message;

moto_measure_t moto_pit;	 					// pitch
moto_measure_t moto_yaw;	 					// yaw
moto_measure_t moto_fric[Fric_Wheel_Count]; 	// 水平摩擦轮
//**********pitch/yaw期望遥控器指针解码******
float angle;
float *p_angle = &angle;

//***************************************
gimbal_state_t gimbal_state;   	// 遥控器yaw状态获取
uint8_t test[8];			  	// 垂直摩擦轮发射标志位
float shoot_42mm_speed; 		// 42mm弹丸初速度


static void STD_CAN_RxCpltCallback(CAN_TypeDef *_hcan, CanRxMsg *message)
{
	if (_hcan == CAN1) // can1中断
	{
		switch (message->StdId)
		{
		case RxCAN_YAW_MOTOR_ID: // yaw电机
		{
			encoder_data_handler(&moto_yaw, message);
			err_detector_hook(GIMBAL_YAW_OFFLINE);
		}
		break;

		case RxCAN_CHASSIS_TO_GIMBAL_ID:
		{
			judge_recv_mesg.game_robot_state.robot_id = (enemy_color_t)rx1_message.Data[0]; // 敌方ID，由裁判系统读取
			shoot_42mm_speed = (float)rx1_message.Data[1];
			judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value = rx1_message.Data[2] << 8 | rx1_message.Data[3];
			judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit = rx1_message.Data[4]<<8 | rx1_message.Data[5];
			judge_recv_mesg.game_state.game_type = rx1_message.Data[6];
		}
		break;
		
		default:
		{
		}
		break;
		}
	}
	else // can2中断
	{
		switch (message->StdId)
		{

		/***摩擦轮***/
		case RxCAN_FRIC_M1D_ID:
		case RxCAN_FRIC_M2R_ID:
		case RxCAN_FRIC_M3L_ID:
		{
			static uint8_t i = 0;
			// 处理电机ID号
			i = message->StdId - RxCAN_FRIC_M1D_ID;
			// 处理电机数据宏函数
			moto_fric[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_fric[i], message) : encoder_data_handler(&moto_fric[i], message);
			err_detector_hook(FRI_MOTO1_OFFLINE + i);
		}
		break;

		case RxCAN_PIT_MOTOR_ID:
		{
			encoder_data_handler(&moto_pit, message);
			err_detector_hook(GIMBAL_PIT_OFFLINE);
		}
		break;
		
		break;
		
		default:
		{
		}
		break;
		}
	}
	
}

void encoder_data_handler(moto_measure_t *ptr, CanRxMsg *message)
{
	ptr->last_ecd = ptr->ecd;
	ptr->ecd = (uint16_t)(message->Data[0] << 8 | message->Data[1]);

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
	ptr->total_angle = ptr->total_ecd / ENCODER_ANGLE_RATIO;

	// 达妙电机一拖四需要区分 接收的转速需要转换
	if (ptr == &moto_pit)
	{
		ptr->speed_rpm = ((int16_t)((message->Data[2] << 8 | message->Data[3])) / 100.0f);
	}
	else
		ptr->speed_rpm = (int16_t)(message->Data[2] << 8 | message->Data[3]);
		ptr->given_current = (int16_t)(message->Data[4] << 8 | message->Data[5]);
}
/**
 * @brief     get motor initialize offset value
 * @param     ptr: Pointer to a moto_measure_t structure
 * @retval    None
 * @attention this function should be called after system can init
 */
void get_moto_offset(moto_measure_t *ptr, CanRxMsg *message)
{
	ptr->ecd = (uint16_t)(message->Data[0] << 8 | message->Data[1]);
	ptr->offset_ecd = ptr->ecd;
}

//**************************************************************************************//
/*
 填充大疆电机电流数据到CAN消息
 */
static void fill_DJImotor_data(CanTxMsg *msg, int16_t data1, int16_t data2,
                            	int16_t data3, int16_t data4)
{
    if (msg == NULL) return;
    msg->Data[0] = (uint8_t)(data1 >> 8);
    msg->Data[1] = (uint8_t)(data1 & 0xFF);
    msg->Data[2] = (uint8_t)(data2 >> 8);
    msg->Data[3] = (uint8_t)(data2 & 0xFF);
    msg->Data[4] = (uint8_t)(data3 >> 8);
    msg->Data[5] = (uint8_t)(data3 & 0xFF);
    msg->Data[6] = (uint8_t)(data4 >> 8);
    msg->Data[7] = (uint8_t)(data4 & 0xFF);
}
/*******************************************CAN2*****************************************************/
// DM_pitch ID 1
void send_gimbal_pit_cur(int16_t pit_iq) //!!!
{
	CanTxMsg TxMessage;
	TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_PIT_DM_ID,0x02);
	
	TxMessage.Data[0] = pit_iq;
	TxMessage.Data[1] = pit_iq >> 8; // 达妙4310高八位放在Data[1],注意别和大疆的搞错了

	CAN_Transmit(CAN2, &TxMessage);
}
// 三摩擦轮
void send_fric_cur(int16_t iq1, int16_t iq2, int16_t iq3) 
{
	CanTxMsg TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_FRIC_MOTOR_ID,0x08);

	fill_DJImotor_data(&TxMessage, iq1, iq2, iq3, 0);

	CAN_Transmit(CAN2, &TxMessage);
}
/*******************************************CAN1*****************************************************/
//	yaw	ID 6
void send_gimbal_yaw_cur(int16_t yaw_iq) //!!!
{
	CanTxMsg TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_YAW_MOTOR_ID,0x08);
	
	fill_DJImotor_data(&TxMessage, 0, yaw_iq, 0, 0);
	CAN_Transmit(CAN1, &TxMessage);
}
// 发送陀螺仪yaw角度
void send_yaw_gyro_angle(float yaw_gyro_angle) //!!!
{
	unsigned char *yaw;
	yaw = (unsigned char *)&yaw_gyro_angle;
	
	CanTxMsg TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_YAW_GYRO_ANGLE_ID,0x08);
	
	TxMessage.Data[0] = *yaw;
	TxMessage.Data[1] = *(yaw+1);
	TxMessage.Data[2] = *(yaw+2);
	TxMessage.Data[3] = *(yaw+3);

	CAN_Transmit(CAN1, &TxMessage);
}
// 云台传到底盘rc与标志位值
void gimbal_to_chassis_rc(void)
{
	CanTxMsg TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_TO_CHASSIS_RC_ID,0x08);
	
	TxMessage.Data[0] = (rc.kb.bit.Z<<7)|(rc.kb.bit.CTRL<<6)|(rc.kb.bit.SHIFT<<5)|(rc.image.pause<<4)|(rc.image.key_l<<3)|(rc.image.key_r<<2)|(rc.image.sw);
	TxMessage.Data[1] = (rc.kb.bit.D<<7)|(rc.kb.bit.S<<6)|(rc.kb.bit.A<<5)|(rc.kb.bit.W<<4)|(rc.sw2<<2)|(rc.sw1);
	TxMessage.Data[2] = (rc.remote_flag<<7)|(shoot.loader_fire_allow<<6)|(gimbal_turn_flag<<5)|global_mode<<3|gimbal.separt_ctrl<<2|gimbal.dodge_ctrl<<1|shoot.fric_wheel_run;
	TxMessage.Data[3] = rc.ch1>>8;
	TxMessage.Data[4] = rc.ch1;
	TxMessage.Data[5] = rc.ch2>>8;
	TxMessage.Data[6] = rc.ch2;
	TxMessage.Data[7] = RC_iw_flag;
	
	CAN_Transmit(CAN1, &TxMessage);
}

/*
函数名：send_to_chassis
参数：
pit——pit相对角度，dis——测距距离，shoot_ready_flag——轻触开关是否触碰
*/
void send_to_chassis(float pit, float dis, uint8_t shoot_ready_flag)
{
	float pit_shoot;
	unsigned char *p1;
	p1 = (unsigned char *)&pit_shoot; // p1——包含pit和shoot_ready_flag信息
	unsigned char *p2;
	p2 = (unsigned char *)&dis; // p2——包含dis信息
	/**由于字节长度受限，把pit相对角与轻触开关合起来（pit不会超过100度，100作为轻触开关标志位）
	轻触开关碰到，pit 10度时，发送 110；轻触开关没碰到，pit 10度，发送10；
	轻触开关碰到，pit -10度，发送-110；轻触开关没碰到，pit -10，发送-10**/
	if (shoot_ready_flag)
	{
		if (pit >= 0)
			pit_shoot = pit + 100;
		else if (pit < 0)
			pit_shoot = pit - 100;
	}
	else
		pit_shoot = pit;
	CanTxMsg TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_TO_CHASSIS_ID,0x08);
	
	TxMessage.Data[0] = *p1;
	TxMessage.Data[1] = *(p1 + 1);
	TxMessage.Data[2] = *(p1 + 2);
	TxMessage.Data[3] = *(p1 + 3);
	TxMessage.Data[4] = *p2;
	TxMessage.Data[5] = *(p2 + 1);
	TxMessage.Data[6] = *(p2 + 2);
	TxMessage.Data[7] = *(p2 + 3);

	CAN_Transmit(CAN1, &TxMessage);
}
// 让底盘完全独立于云台
void send_yaw_to_chassis(float total, float offset)
{
	unsigned char *p1;
	p1 = (unsigned char *)&total; // p1——包含pit和shoot_ready_flag信息
	unsigned char *p2;
	p2 = (unsigned char *)&offset; // p2——包含dis信息
	
	CanTxMsg TxMessage = (CanTxMsg)CAN_STD_DATA_FRAME(TxCAN_TO_CHASSIS_YAW_ID,0x08);
	
	TxMessage.Data[0] = *p1;
	TxMessage.Data[1] = *(p1 + 1);
	TxMessage.Data[2] = *(p1 + 2);
	TxMessage.Data[3] = *(p1 + 3);
	TxMessage.Data[4] = *p2;
	TxMessage.Data[5] = *(p2 + 1);
	TxMessage.Data[6] = *(p2 + 2);
	TxMessage.Data[7] = *(p2 + 3);

	CAN_Transmit(CAN1, &TxMessage);
}

// can1中断
void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
		STD_CAN_RxCpltCallback(CAN1, &rx1_message);
	}
}

// can2中断
void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
		STD_CAN_RxCpltCallback(CAN2, &rx2_message);
	}
}
