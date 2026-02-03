#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"
#include "info_get_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_vofa.h"
#include "comm_task.h"
#include "detect_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "sys_config.h"
#include "pc_rx_data.h"
#include "pid.h"
#include "stdlib.h"
#include "stdlib.h" //abs()函数
#include "math.h"	//fabs()函数
#include "ladrc.h"
#include "shoot_task.h"
#include "debug_wave.h"

UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

gimbal_t gimbal;					// 云台电机pitch/yaw控制所需变量
ramp_t pit_ramp;					// pitch归中斜坡函数
ramp_t yaw_ramp;					// yaw归中斜坡函数
extern float angle_yaw;				// yaw角度期望
extern float angle_pit;				// pitch角度期望
extern gimbal_state_t gimbal_state; // yaw当前状态是否有输入
extern signed char i_temp;			// 用于记录yaw轴转了多少圈

/* 归中pid参数 */
float pit_pid_init[6] = {20, 0.1f, 0, 150, 0, 0}; // pit轴pid
float yaw_pid_init[6] = {25, 0, 0, 500, 0, 0};	  // yaw轴pid

/****************************************************************
					YAW轴开环函数
2025.6.19
					 0.001323             1
  	G(s) = ----------   =  -------------  
					  s+5.719        755.86s + 4322.75 

Wc截止频率设为30Hz
速度环PI参数设定：Kp=755.86*Wc Ki=4322.75*Wc

*****************************************************************/
#define PID_Ts 0.003 
#define Wc_spd_loop 20
#define Kp_spd_loop 755.86*Wc_spd_loop
#define Ki_spd_loop 4322.75*Wc_spd_loop*PID_Ts

/* 正常模式pid参数 */
float pit_pid[6] = {0.6, 0, 0, 4500, 25, 0};	 // pit轴pid60,163
float yaw_pid[6] = {0.15, 0.001, 0, 50000, 0.1, 0}; // yaw轴pid
/* 分离模式pid参数 */
float pit_seprarte_pid[6] = {1, 0, 0, 2500, 20, 0};	 // pit轴pid60,163
float yaw_seprarte_pid[6] = {1, 0, 0, Kp_spd_loop, Ki_spd_loop, 0}; // yaw轴pid
/*掉头pid*/
float yaw_turn_pid[6] = {0.07, 0, 0, 20000, 0, 0}; // yaw轴pid
/* 自瞄模式pid参数 */
//float pit_track_pid[6] = {0.5, 0., 0, 6000, 0, 0};
//float yaw_track_pid[6] = {0.3, 0, 0, 16000, 0, 0};

float pit_track_pid[6] = {0.4, 0.001, 0, 4000, 0, 0};
float yaw_track_pid[6] = {0.15, 0, 0, 20000, 0, 0};

/* 输出限幅 */
float pit_limit[4] = {30, 10, 10000, 2000}; // 角度环输出目标速度限幅（云台转动速度最大到600左右）， 角度环积分限幅， 速度环目标速度限幅（6020最大发送电压25000）， 速度环积分限幅
float yaw_limit[4] = {100, 50, 16384, 5000}; // 角度环输出目标速度限幅（云台转动速度最大到600左右）， 角度环积分限幅， 速度环目标速度限幅（6020最大发送电压25000）， 速度环积分限幅
//!!!电机方向与电机安装方向有关！！！
float pit_dir = -1.f; // 目前是在右边，底部向<---
float yaw_dir = -1.f; // 目前是反装,底部向↑


/* 前馈控制器，使用时更改控制器宏定义 */
Easy_FFC_t yaw_easy_ffc;
Easy_FFC_t pit_easy_ffc;
float pit_track_ctrl;
float yaw_track_ctrl;
float pit_pid_ctrl;
float yaw_pid_ctrl;

FFC p_ffc;
FFC y_ffc;
/* PITCH限位宏 */
#define PIT_ANGLE_MAX 40
#define PIT_ANGLE_MIN -11
/* YAW限位宏 */
#define YAW_ANGLE_MAX 50
#define YAW_ANGLE_MIN -50

/*快速归中宏定义*/
#define FAST_CALIBRATE 0
/*控制器选择宏定义*/
#define ANGLE_PID 1
#define SPEED_PID 1
#define FUZZY_SPEED_PID 0
#define EASY_FFC_PID 0
/*wasd控制云台倍率*/
#define YAW_ratio 0.004f
#define PIT_ratio 0.004f

FirstOrderFilter Filter_valua;
float Gravity_valua ;

float debug_track;

void gimbal_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)INFO_GET_GIMBAL_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{

			if (Signal & INFO_GET_GIMBAL_SIGNAL)
			{

				if (gimbal_mode == GIMBAL_INIT) // yaw
				{
					/* pit 轴电机的PID参数 */
					PID_Struct_Init(&pid_pit, pit_pid_init[0], pit_pid_init[1], pit_pid_init[2], pit_limit[0], pit_limit[1], DONE);
					PID_Struct_Init(&pid_pit_spd, pit_pid_init[3], pit_pid_init[4], pit_pid_init[5], pit_limit[2], pit_limit[3], DONE);

					/* yaw 轴电机的PID参数 */
					PID_Struct_Init(&pid_yaw, yaw_pid_init[0], yaw_pid_init[1], yaw_pid_init[2], yaw_limit[0], yaw_limit[1], DONE);
					PID_Struct_Init(&pid_yaw_spd, yaw_pid_init[3], yaw_pid_init[4], yaw_pid_init[5], yaw_limit[2], yaw_limit[3], DONE);
				}
				else if (gimbal_mode != GIMBAL_TRACK_ARMOR && gimbal_mode != GIMBAL_SEPARATE_MODE)
				{
					
					/* pit 轴电机的PID参数 */
					PID_Struct_Init(&pid_pit, pit_pid[0], pit_pid[1], pit_pid[2], pit_limit[0], pit_limit[1], DONE);
					PID_Struct_Init(&pid_pit_spd, pit_pid[3], pit_pid[4], pit_pid[5], pit_limit[2], pit_limit[3], DONE);
					
					/* yaw 轴电机的PID参数 */		
					if(gimbal_turn_ready)
					{
						PID_Clear(&pid_yaw);
						PID_Clear(&pid_yaw_spd);
						PID_Struct_Init(&pid_yaw, yaw_turn_pid[0], yaw_turn_pid[1], yaw_turn_pid[2], yaw_limit[0], yaw_limit[1], DONE);
						PID_Struct_Init(&pid_yaw_spd, yaw_turn_pid[3], yaw_turn_pid[4], yaw_turn_pid[5], yaw_limit[2], yaw_limit[3], DONE);
					}
					else
					{
						PID_Clear(&pid_yaw);
						PID_Clear(&pid_yaw_spd);
						PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], yaw_limit[0], yaw_limit[1], DONE);
						PID_Struct_Init(&pid_yaw_spd, yaw_pid[3], yaw_pid[4], yaw_pid[5], yaw_limit[2], yaw_limit[3], DONE);
					}
			
				}
				else if(gimbal_mode == GIMBAL_SEPARATE_MODE)
				{
					/* pit 轴电机的PID参数 */
					PID_Struct_Init(&pid_pit, pit_seprarte_pid[0], pit_seprarte_pid[1], pit_seprarte_pid[2], pit_limit[0], pit_limit[1], DONE);
					PID_Struct_Init(&pid_pit_spd, pit_seprarte_pid[3], pit_seprarte_pid[4], pit_seprarte_pid[5], pit_limit[2], pit_limit[3], DONE);

					/* yaw 轴电机的PID参数 */
					PID_Struct_Init(&pid_yaw, yaw_seprarte_pid[0], yaw_seprarte_pid[1], yaw_seprarte_pid[2], yaw_limit[0], yaw_limit[1], DONE);
					PID_Struct_Init(&pid_yaw_spd, yaw_seprarte_pid[3], yaw_seprarte_pid[4], yaw_seprarte_pid[5], yaw_limit[2], yaw_limit[3], DONE);
				
				}
				else if (gimbal_mode == GIMBAL_TRACK_ARMOR)
				{
					/* pit 轴电机的PID参数 */
					PID_Struct_Init(&pid_pit, pit_track_pid[0], pit_track_pid[1], pit_track_pid[2], pit_limit[0], pit_limit[1], DONE);
					PID_Struct_Init(&pid_pit_spd, pit_track_pid[3], pit_track_pid[4], pit_track_pid[5], pit_limit[2], pit_limit[3], DONE);

					/* yaw 轴电机的PID参数 */
					PID_Struct_Init(&pid_yaw, yaw_track_pid[0], yaw_track_pid[1], yaw_track_pid[2], yaw_limit[0], yaw_limit[1], DONE);
					PID_Struct_Init(&pid_yaw_spd, yaw_track_pid[3], yaw_track_pid[4], yaw_track_pid[5], yaw_limit[2], yaw_limit[3], DONE);
				}
				 
				if (gimbal_mode != GIMBAL_RELEASE) // 当云台非RELEASE
				{
					switch (gimbal_mode)
					{
					/*云台底盘归中模式*/
					case GIMBAL_INIT:
					{
						if (last_gimbal_mode != GIMBAL_INIT)
						{
							PID_Clear(&pid_pit);
							PID_Clear(&pid_yaw);
							PID_Clear(&pid_pit_spd);
							PID_Clear(&pid_yaw_spd);
						}
						init_mode_handler(); // 云台回中
					}
					break;
					/*云台底盘跟随模式*/
					case GIMBAL_NORMAL_MODE:
					{
						if (last_gimbal_mode != GIMBAL_NORMAL_MODE)
						{
							PID_Clear(&pid_pit);
							PID_Clear(&pid_yaw);
							PID_Clear(&pid_pit_spd);
							PID_Clear(&pid_yaw_spd);
						}
						Nomarl_Dodge_Seprarte_handler();
					}
					break;
					/*云台底盘分离模式*/
					case GIMBAL_SEPARATE_MODE:
					{
						if (last_gimbal_mode != GIMBAL_SEPARATE_MODE)
						{
							PID_Clear(&pid_pit);
							PID_Clear(&pid_yaw);
							PID_Clear(&pid_pit_spd);
							PID_Clear(&pid_yaw_spd);
						}
						Nomarl_Dodge_Seprarte_handler();
					}
					break;
					/*小陀螺模式*/
					case GIMBAL_DODGE_MODE:
					{
						if (last_gimbal_mode != GIMBAL_DODGE_MODE)
						{
							PID_Clear(&pid_pit);
							PID_Clear(&pid_yaw);
							PID_Clear(&pid_pit_spd);
							PID_Clear(&pid_yaw_spd);
						}
						Nomarl_Dodge_Seprarte_handler();
					}
					break;
					/*自瞄模式*/
					case GIMBAL_TRACK_ARMOR:
					{
						if (last_gimbal_mode != GIMBAL_TRACK_ARMOR)
						{
							PID_Clear(&pid_pit);
							PID_Clear(&pid_yaw);
							PID_Clear(&pid_pit_spd);
							PID_Clear(&pid_yaw_spd);
						}
						track_aimor_handler();
					}
					break;

					default:
					{
					}
					break;
					}
				}
				else // gimbal_mode == REALEASE
				{
					memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur)); // 云台电机无力
					gimbal.state = GIMBAL_INIT_NEVER;						   // 云台预备归中
					
				}

//				update_sine_wave(&gimbal.pid.yaw_angle_ref);
//				vofa_debug[0] = gimbal.pid.pit_angle_ref;
//				vofa_debug[1] = gimbal.pid.pit_angle_fdb;
//				vofa_debug[2] =	gimbal.pid.pit_spd_ref;
//				vofa_debug[3] = gimbal.pid.pit_spd_fdb;

//				vofa_debug[1] = glb_cur.fric_cur[0];
//				vofa_debug[2] = glb_cur.fric_cur[1];
//				vofa_debug[3] = glb_cur.fric_cur[3];
//				vofa_debug[3] = glb_cur.fric_cur[3];
//				vofa_debug[1] = gimbal.pid.pit_angle_fdb;
//				vofa_debug[0] = moto_pit.given_current;

				
//				JustFloat_Send(vofa_debug, 2, USART1);
				
				
				// PID角度环计算
				pid_calc(&pid_yaw, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_ref);
				pid_calc(&pid_pit, gimbal.pid.pit_angle_fdb, gimbal.pid.pit_angle_ref);

// PID角度环输出值作为速度环给定值——串级控制
#if EASY_FFC_PID // 速度环前馈期望
				{
					Easy_FFC_Calc(&yaw_easy_ffc, yaw_easy_ffc.Kc, gimbal.pid.yaw_angle_ref);
					Easy_FFC_Calc(&pit_easy_ffc, pit_easy_ffc.Kc, gimbal.pid.pit_angle_ref);

					gimbal.pid.yaw_spd_ref = pid_yaw.out + yaw_easy_ffc.out;
					gimbal.pid.pit_spd_ref = pid_pit.out + pit_easy_ffc.out;
				}
#elif ANGLE_PID // 普通速度环期望
				{
					gimbal.pid.yaw_spd_ref = pid_yaw.out;//Sweep()
					gimbal.pid.pit_spd_ref = pid_pit.out;// 0.5pid_pit.out
				}
#endif
				// PID速度环反馈给定
				gimbal.pid.yaw_spd_fdb = gimbal.sensor.yaw_palstance;// gimbal.sensor.yaw_palstance;moto_yaw.speed_rpm;
				gimbal.pid.pit_spd_fdb = gimbal.sensor.pit_palstance;

// 模糊pid
#if FUZZY_SPEED_PID // 模糊pid计算速度环输出值
				{
					// PID速度环计算——模糊PID控制器
					fuzzy_pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
					fuzzy_pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
				}
#elif SPEED_PID
				{
					// PID速度环计算——普通PID控制器
					pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);
					pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);
				}
#endif

				/*自瞄模式pit/yaw转矩电流*/
				pit_track_ctrl = pid_pit_spd.out; // getFeedforwardControl(&p_ffc, gimbal.pid.pit_angle_ref) +pid_pit_spd.out;//前馈
				yaw_track_ctrl = pid_yaw_spd.out; // getFeedforwardControl(&y_ffc, gimbal.pid.yaw_angle_ref) + pid_yaw_spd.out;

				/*正常模式pit/yaw转矩电流*/
				pit_pid_ctrl = pid_pit_spd.out;
				yaw_pid_ctrl = pid_yaw_spd.out;
				
				Gravity_valua = filter_update(&Filter_valua,Gravity_compensation(gimbal.pid.pit_angle_fdb,gimbal.pid.pit_spd_fdb));
				if (1) // 云台电机未掉线，则将PID计算值输出gimbal_is_controllable()
				{
					if (gimbal_mode == GIMBAL_TRACK_ARMOR)
					{
						glb_cur.gimbal_cur[YAW] = yaw_dir * yaw_track_ctrl;
						glb_cur.gimbal_cur[PITCH] = pit_dir * (filter_update(&Filter_valua,pit_pid_ctrl) );
						//memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur)); + Gravity_valua
					}
					else if (gimbal_mode == GIMBAL_DODGE_MODE)
					{
						glb_cur.gimbal_cur[YAW] = yaw_dir * yaw_pid_ctrl + 6000;
						if (glb_cur.gimbal_cur[YAW] > 16384)
							glb_cur.gimbal_cur[YAW] = 16384;
						glb_cur.gimbal_cur[PITCH] = pit_dir * (filter_update(&Filter_valua,pit_pid_ctrl) );
					}//+ Gravity_valua
					else
					{
						glb_cur.gimbal_cur[YAW] = yaw_dir * yaw_pid_ctrl;
						glb_cur.gimbal_cur[PITCH] = pit_dir * (filter_update(&Filter_valua,pit_pid_ctrl) );
					}// + Gravity_valua
				}
				else
				{
					memset(glb_cur.gimbal_cur, 0, sizeof(glb_cur.gimbal_cur));
					gimbal_mode = GIMBAL_RELEASE; //
					pid_trigger_angle.iout = 0;
				}

				get_last_mode(); // 获取上一次的云台模式 last_gimbal_mode

				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)GIMBAL_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
			}
		}

		gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

/*云台初始化*/
void gimbal_param_init(void)
{
	memset(&gimbal, 0, sizeof(gimbal_t));

	gimbal.state = GIMBAL_INIT_NEVER; // 开发板刚上电云台要进入归

	/* pit 轴电机的PID参数 */
	PID_Struct_Init(&pid_pit, pit_pid_init[0], pit_pid_init[1], pit_pid_init[2], pit_limit[0], pit_limit[1], INIT);
	PID_Struct_Init(&pid_pit_spd, pit_pid_init[3], pit_pid_init[4], pit_pid_init[5], pit_limit[2], pit_limit[3], INIT);

	/* yaw 轴电机的PID参数 */
	PID_Struct_Init(&pid_yaw, yaw_pid[0], yaw_pid[1], yaw_pid[2], yaw_limit[0], yaw_limit[1], INIT);
	PID_Struct_Init(&pid_yaw_spd, yaw_pid[3], yaw_pid[4], yaw_pid[5], yaw_limit[2], yaw_limit[3], INIT);

	/*前馈控制器初始化*/
	Easy_FFC_Init(&pit_easy_ffc, 600);
	Easy_FFC_Init(&yaw_easy_ffc, 450);
	/* FFC */
	initFeedforwardParam(&p_ffc, 210, 20);
	initFeedforwardParam(&y_ffc, 210, 20);
	
	filter_init_obc(&Filter_valua,0,0.8);
}
	
/* 归中模式，先归PIT再归YAW */
static void init_mode_handler(void)
{
	// 将当前角度赋给反馈值
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
	// 目标值设为零（斜坡函数需要时可加）
	gimbal.pid.pit_angle_ref = 0;
	// 将当前相对角度赋给反馈值
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
	// 目标值设为当前相对角度
	gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_total_angle;

	gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;

	gimbal.state = GIMBAL_INIT_DONE;
}

/*判断yaw轴是否有输入*/
/*判断yaw轴是否有输入*/
static gimbal_state_t remote_is_action(void)
{	
	if((gimbal_mode == GIMBAL_SEPARATE_MODE) && ((rc.kb.bit.D) || (rc.kb.bit.A)) ) // 分离模式下AD的控制判断
	{
		return IS_ACTION;
	}
	else if ((abs(rc.ch3) >= 10) || (abs(rc.mouse.x) >= 1))
	{
		return IS_ACTION;
	}
	else
	{
		return NO_ACTION;
	}
}
// 正常模式
/* creat manual control funtion.  add time:2025.1.5*/
static void manual_control_funtion(gimbal_status NOW_MODE)
{
	
	gimbal.state = remote_is_action(); // 判断yaw轴是否有输入

	if ((gimbal.last_state == NO_ACTION) && (gimbal.state == NO_ACTION))
	{
		switch (gimbal.trace_state)
		{
			case REF_CONFIRM: // 判断正反转
			{
				
				if (gimbal.sensor.yaw_palstance > 0) // 判断当前为正转
					gimbal.trace_state = REF_ADD;
				else if (gimbal.sensor.yaw_palstance < 0) // 判断当前为反转
					gimbal.trace_state = REF_SUB;
				else
				{
					gimbal.trace_state = REF_KEEP;
				}
				gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 刷新底盘0轴
				
			}
			break;
		
			case REF_ADD: // 正转，目标值向上阶跃
			{
				if ((gimbal.sensor.yaw_palstance < 0.1f)) // || (HAL_GetTick() - confirm_time >= 100))
				{
					gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
					gimbal.trace_state = REF_KEEP; // 反馈跟随目标阶段
				}
				gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 刷新底盘0轴
			}
			break;
			
			case REF_SUB: // 反转，目标值向下阶跃
			{
				if ((gimbal.sensor.yaw_palstance > (-0.1f))) // || (HAL_GetTick() - confirm_time >= 100))
				{
					gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
					gimbal.trace_state = REF_KEEP; // 反馈跟随目标阶段
				}
				gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 刷新底盘0轴
			}
			break;
			
			case REF_KEEP:
			{
				gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 刷新底盘0轴
			}
			default:
			break; // 完全停止后不刷新底盘0轴，此时底盘云台分开锁死
		}
	}
	else if (gimbal.last_state == IS_ACTION && gimbal.state == NO_ACTION) // 进入目标值跟踪阶段，消除动态超调
	{
		gimbal.trace_state = REF_CONFIRM;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 云台停止后刷新底盘0轴
	}
	else
	{
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle; // 在yaw轴动的情况下刷新0轴
		gimbal.trace_state = REF_KEEP;							 // 反馈跟随目标阶段
	}
	
	/*yaw轴*/
	gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
	if(gimbal_turn_flag)				//一键掉头yaw转180°
		gimbal.pid.yaw_angle_ref = gimbal.sensor.yaw_total_angle + 180;
	if(NOW_MODE == GIMBAL_SEPARATE_MODE)//分离模式下键盘控制
		gimbal.pid.yaw_angle_ref += (-km.yaw_v  * YAW_ratio);
	else
	    gimbal.pid.yaw_angle_ref += ((rm.yaw_v * GIMBAL_RC_MOVE_RATIO_YAW) - km.yaw_v * GIMBAL_PC_MOVE_RATIO_YAW) * 1.4f; // 给定 = 遥控器给定+键盘给定 ，遥控器正负与方向有关

	/*pitch轴*/
	gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
	
	if(gimbal_turn_flag)//一键掉头pitch归中
		gimbal.pid.pit_angle_ref = 0;
	if(NOW_MODE == GIMBAL_SEPARATE_MODE)//分离模式下键盘控制
		gimbal.pid.pit_angle_ref += (km.pit_v * PIT_ratio); // 给定为遥控器数据
	else
		gimbal.pid.pit_angle_ref += (rm.pit_v * GIMBAL_RC_MOVE_RATIO_PIT) + km.pit_v * GIMBAL_PC_MOVE_RATIO_PIT; // 给定 = 遥控器给定+键盘给定 遥控器正负与方向有关
	/* 软件限制pitch轴角度 */
	VAL_LIMIT(gimbal.pid.pit_angle_ref, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
	gimbal.last_state = gimbal.state; // 获取上次输入的状态
}


static void Nomarl_Dodge_Seprarte_handler(void)
{
	if (gimbal_mode == GIMBAL_SEPARATE_MODE)
	{
		//记录yaw轴
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
		gimbal.pid.yaw_angle_ref = gimbal.pid.seprarte_yaw_ref;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;
		//记录pit轴
		gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
		gimbal.pid.pit_angle_ref = gimbal.pid.seprarte_pit_ref;
			
		if(km.yaw_v || km.pit_v)
		{
			manual_control_funtion(gimbal_mode);
			gimbal.pid.seprarte_pit_ref = gimbal.pid.pit_angle_ref;
			gimbal.pid.seprarte_yaw_ref = gimbal.pid.yaw_angle_ref;
		}
	}
	else
	{
		gimbal.pid.seprarte_pit_ref = gimbal.pid.pit_angle_ref;
		gimbal.pid.seprarte_yaw_ref = gimbal.pid.yaw_angle_ref;
		manual_control_funtion(gimbal_mode);
	}
}

/** @brief 	   自瞄模式
**	@attention point重复建系-->gimbal.yaw_angle = gimbal.sensor.yaw_gyro_angle;！！！
**  @author
**/
LADRC_NUM Vision_Angle_Pit =
	{
		.h = 0.002, // 定时时间及时间步长
		.r = 30,	// 跟踪速度参数
};
LADRC_NUM Vision_Angle_Yaw =
	{
		.h = 0.002, // 定时时间及时间步长
		.r = 30,	// 跟踪速度参数
};

static void track_aimor_handler(void)
{
	pid_yaw.iout = 0; // 清空其他模式yaw角度环iout累加值
	static u8 vision_reletive = 0;				//置1则为相对角，否则为绝对角

	if (last_gimbal_mode != GIMBAL_TRACK_ARMOR && gimbal_mode == GIMBAL_TRACK_ARMOR)
	{
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;
		gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;
		
		gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
		gimbal.pid.pit_angle_ref = gimbal.pid.seprarte_pit_ref;
		
		gimbal.state = NO_ACTION;
		gimbal.last_state = NO_ACTION;
	}

	if (pc_recv_mesg.mode_Union.info.visual_valid == 1)
	{
		
		gimbal.pid.pit_angle_fdb = gimbal.sensor.pit_gyro_angle;
		gimbal.pid.yaw_angle_fdb = gimbal.sensor.yaw_total_angle;  // yaw轴用陀螺仪
		if(vision_reletive)
		{
			gimbal.pid.yaw_angle_ref = pc_recv_mesg.aim_yaw + gimbal.pid.yaw_angle_fdb;
			gimbal.pid.pit_angle_ref = pc_recv_mesg.aim_pitch + gimbal.pid.pit_angle_fdb;
		}
		else
		{
			gimbal.pid.yaw_angle_ref = 360.0f*gimbal.sensor.yaw_cnt + pc_recv_mesg.aim_yaw;
			gimbal.pid.pit_angle_ref = pc_recv_mesg.aim_pitch;
		}
		gimbal.yaw_offset_angle = gimbal.sensor.yaw_total_angle;		
	}	
	/*2025.01.04 new visual invalid processing */
	else if (pc_recv_mesg.mode_Union.info.visual_valid == 0) // 无效时可手动控制，防止变成呆瓜
	{
		manual_control_funtion(gimbal_mode);
		gimbal.pid.seprarte_pit_ref = gimbal.pid.pit_angle_ref;
		gimbal.pid.seprarte_yaw_ref = gimbal.pid.yaw_angle_ref;
	}
}

/*初始化前馈补偿*/
void initFeedforwardParam(FFC *vFFC, float a, float b)
{

	vFFC->a = a;
	vFFC->b = b;
	vFFC->lastRin = 0;
	vFFC->perrRin = 0;
	vFFC->rin = 0;
}

/*实现前馈控制器*/
float getFeedforwardControl(FFC *vFFC, float v) // yaw轴
{
	vFFC->rin = v;
	float result = vFFC->a * (vFFC->rin - vFFC->lastRin) + vFFC->b * (vFFC->rin - 2 * vFFC->lastRin + vFFC->perrRin);
	vFFC->perrRin = vFFC->lastRin;
	vFFC->lastRin = vFFC->rin;
	return result;
}
/*EASY_FFT*/
void Easy_FFC_Init(Easy_FFC_t *easy_ffc, float K)
{
	easy_ffc->Kc = K;
}
void Easy_FFC_Calc(Easy_FFC_t *easy_ffc, float K, float gimbal_expect_ref)
{
	easy_ffc->Outner_Expect[NOW] = gimbal_expect_ref;
	easy_ffc->out = K * (easy_ffc->Outner_Expect[NOW] - easy_ffc->Outner_Expect[LAST]);
	easy_ffc->Outner_Expect[LAST] = easy_ffc->Outner_Expect[NOW];
}

double frequency = 4;	// 频率33.33Hz，对应周期0.03秒
float CYCLE_TIME = 0.1; // 执行周期为0.03秒
#define amplitude 15

void update_sine_wave(float *value)
{
	// 假设正弦波从0开始，每次增加2π/频率
	static double phase = 0.0;
	*value = amplitude * sin(2 * PI * frequency * phase);
	phase += 1 / CYCLE_TIME; // 更新相位
	if (phase >= 1)
		phase -= 1; // 保持相位在0和1之间
}
/*重力补偿*/
float Gravity_compensation(float angle,float w)
{
	float iq;
	float angle_3 = angle*angle*angle;
	float angle_2	= angle*angle;
	float width = 0.2f;

	double iq_up =- 0.02612f*angle_3
				  + 1.385f*angle_2
				  - 17.74f*angle
				  - 1460;

	double iq_down =+ 0.002861f*angle_3
					+ 0.1772f*angle_2
					- 31.12f*angle
					+ 835.5;
	float b = (iq_up + iq_down)/2;
	float k = (iq_up - b)/width;
	if(w >=0 )
	{
		if(w <= width)
			iq = b + k*w;
		else
			iq = iq_up;
	}
	else if(w < 0)
	{
		if(w >= -width)
			iq = b + k*w;
		else
			iq = iq_down;
	}
	return iq;
}
// 初始化滤波器
// initial_value: 初始输出值 (通常设为第一次输入值)
// alpha: 滤波系数 (决定平滑程度)
void filter_init_obc(FirstOrderFilter* filter, float initial_value, float alpha) {
    filter->prev_output = initial_value;
    filter->alpha = alpha;
}
// 执行一步滤波计算
float filter_update(FirstOrderFilter* filter, float input) {
    // 核心公式: output = alpha * input + (1 - alpha) * prev_output
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    
    // 更新状态
    filter->prev_output = output;
    return output;
}
