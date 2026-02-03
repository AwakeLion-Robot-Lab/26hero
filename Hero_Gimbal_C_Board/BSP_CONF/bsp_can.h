/************************************************************************************************************
底盘开发板		——can1：
						接收：
						底盘电机  			CAN_3508_M1_ID       = 0x201,
											CAN_3508_M2_ID       = 0x202,
											CAN_3508_M3_ID       = 0x203,
											CAN_3508_M4_ID       = 0x204,

						拨盘电机 			CAN_TRIGGER_MOTOR_ID  = 0x208,
						电容      			CAN_SUPER_CAP_ID	  = 0x211,
						
						发送：
						底盘电机  			CAN_CHASSIS_ALL_ID   = 0x200,
						拨盘电机 			CAN_TRIG_ALL_ID		 = 0x1ff,
						电容				CAN_CAP_POWER_ID	 = 0x210,
						
				——can2：
						接收：
						yaw轴电机 			CAN_YAW_MOTOR_ID     = 0x20a,
						云台开发板 			GIMBAL_MASTER_ID     = 0x101,
						
						发送：
						云台开发板 			CAN_ANGLE_REF_ID 	 = 0x306,
											CAN_MODE_ID			 = 0x307,
云台开发板		——can1：
						接收：
						yaw轴电机 			CAN_YAW_MOTOR_ID     = 0x20a,
						底盘开发板 			CAN_ANGLE_REF_ID 	 = 0x306,
											CAN_MODE_ID			 = 0x307,
						发送:
						yaw轴电机  			CAN_GIMBAL_ALL_ID    = 0x2ff,
						底盘开发板 			CAN_TO_CHASSIS_ID    = 0x101,					 
				——can2：
						接收：
						摩擦轮 				CAN_FRIC_M1_ID       = 0x205,
											CAN_FRIC_M2_ID       = 0x206,
						pit轴电机 			CAN_PIT_MOTOR_ID     = 0x20b,
						上拨盘电机 			CAN_TRIG_ID          = 0x208,
						
						发送：
						pit轴电机  			CAN_GIMBAL_ALL_ID    = 0x2ff,
						摩擦轮、上拨盘  		CAN_FRIC_ALL_ID      = 0x1ff,
**************************************************************************************************************/
#ifndef _bsp_can_H
#define _bsp_can_H


#include "stm32f4xx.h"
#include "sys_config.h"
#define FILTER_BUF 5
/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
/* 摩擦轮个数 */
#define Fric_Wheel_Count 3
/* CAN send and receive ID */
typedef enum
{
	RxCAN_YAW_MOTOR_ID   		= 0x20A,
	RxCAN_CHASSIS_TO_GIMBAL_ID  = 0x307,
	
	TxCAN_PIT_DM_ID 			= 0x3FE,
	TxCAN_YAW_MOTOR_ID     		= 0x2FE,//!!!

	TxCAN_YAW_GYRO_ANGLE_ID  	= 0x30b,
	
	TxCAN_TO_CHASSIS_RC_ID		= 0x30A,
	TxCAN_TO_CHASSIS_ID    		= 0x101,
	TxCAN_TO_CHASSIS_YAW_ID    	= 0x102,
		
	CAN_SHOOT_CMD_ID			= 0x308,
	
	CAN_GIMBAL_ALL_ID   		= 0x2ff,//!!!
} can1_msg_id_e;

typedef enum //!!!
{
	RxCAN_PIT_MOTOR_ID     	= 0x301,
	
    RxCAN_FRIC_M1D_ID      = 0x201,//下
	RxCAN_FRIC_M2R_ID      = 0x202,//右上
    RxCAN_FRIC_M3L_ID      = 0x203,//左上
	
	TxCAN_FRIC_MOTOR_ID		= 0x200,
	
	CAN_TRIG_ID          = 0x208,
	CAN_FRIC_ALL_ID      = 0x200, //!!! 
	
} can2_msg_id_e;

typedef struct
{
  uint16_t ecd;
  uint16_t last_ecd;
  
  int16_t  speed_rpm;
  int16_t  given_current;

  int32_t  round_cnt;
  int32_t  total_ecd;
  int32_t  total_angle;
  
  uint16_t offset_ecd;
  uint32_t msg_cnt;
  
  int32_t  ecd_raw_rate;
  int32_t  rate_buf[FILTER_BUF];
  uint8_t  buf_cut;
  int32_t  filter_rate;
} moto_measure_t;

extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_pit;
extern moto_measure_t moto_yaw;
extern moto_measure_t moto_fric[Fric_Wheel_Count];

extern float shoot_42mm_speed;
extern uint8_t receive_image_angle[2];

void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message);
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message);


//void send_gimbal_cur(int16_t yaw_iq, int16_t pit_iq, trigger_iq);
void send_gimbal_yaw_cur(int16_t yaw_iq); //!!!
void send_gimbal_pit_cur(int16_t pit_iq); //!!!
void send_fric_cur(int16_t iq1, int16_t iq2, int16_t iq3);
void send_magnifying_glass_cur(int16_t glass_iq);
void send_to_chassis(float pit,float dis,uint8_t shoot_ready_flag);
void send_yaw_to_chassis(float total,float offset);
void gimbal_to_chassis_rc(void);

#endif 

