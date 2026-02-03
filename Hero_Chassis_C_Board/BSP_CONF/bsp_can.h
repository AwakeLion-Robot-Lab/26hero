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
#include "chassis_task.h"
#define FILTER_BUF 5
#define Speed_Data 0x01
#define BusCurrent 0x02
#define QValue 0x03
#define RotorPosition 0x04
#define FaultInfo 0x05
#define WarningInfo 0x06
#define MOSTemperature 0x07
#define MotorWindingTemperature 0x08
#define CurrentMode 0x09
#define SystemVoltage 10
#define RotationCount 11
#define SystemState 12
#define AbsolutePosition 13
#define MaxPhaseCurrent 14

#define Mode_Current 0
#define	Mode_Speed 1
#define Mode_Position 2


#define DM_P_MIN -12.5
#define DM_P_MAX  12.5
#define DM_V_MIN -25.0
#define DM_V_MAX  25.0
#define DM_T_MIN -200.0
#define DM_T_MAX  200.0
#define DM_Kp_MAX 500
#define DM_Kp_MIN 0
#define DM_Kd_MAX 5
#define DM_Kd_MIN 0

/* 电机编码值 和 角度（度） 的比率 */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

/* CAN send and receive ID */
typedef enum
{
	CAN_CHASSIS_ALL_ID   = 0x200,
	CAN_3508_M1_ID       = 0x201,//RF
	CAN_3508_M2_ID       = 0x202,//LF
	CAN_3508_M3_ID       = 0x203,//LB
	CAN_3508_M4_ID       = 0x204,//RB
	
  	CAN_TRIG_ALL_ID		  = 0x1ff,
	CAN_TRIGGER_MOTOR_ID  = 0x208,//REAR

	CAN_CAP_POWER_ID	   	= 0x210,
	CAN_SUPER_CAP_ID	   	= 0x211,
} can1_msg_id_e;

typedef enum //!!!
{
	CAN_YAW_MOTOR_ID     = 0x20a,
	GIMBAL_MASTER_ID     = 0x101,
	GIMBAL_YAW_MASTER_ID = 0x102,	

	CAN_ANGLE_REF_ID 			= 0x306,
	CAN_JUDGE_TO_GIMBAL_ID		= 0x307,
	CAN_SHOOT_CMD				= 0x308,
	gimbal_to_chassis_rc_ID 	= 0x30a,
	gimbal_to_chassis_yaw_ID	= 0x30b,
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
extern moto_measure_t moto_loader;

void encoder_data_handler(moto_measure_t* ptr, CanRxMsg *message);
void get_moto_offset(moto_measure_t* ptr, CanRxMsg *message);

void BM1010b_encoder_data_handler(chassis_joint_t* chassis_joint,CanRxMsg *message);
void send_chassis_cur(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void Motor10010B_Enable(void);
void Motor10010B_Current(int16_t Current1 ,int16_t Current2,int16_t Current3,int16_t Current4);
void Motor10010B_FeedBackmode(uint8_t ID,uint8_t Interval,uint8_t Data0,uint8_t Data1,uint8_t Data2,uint8_t Data3);//Interval为反馈时间时间间隔(1-255ms)
void Motor_Init(void);
void send_trig_cur1(int16_t iq8);
void send_cap_power_can(uint16_t tempower);
void send_judge_to_gimbal(void);
void pit_encode(float *angle_flag);
void send_bullet_shoot_cmd(void);
#endif 

