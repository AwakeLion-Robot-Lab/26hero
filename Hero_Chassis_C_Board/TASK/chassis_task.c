#include "chassis_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "pid.h"
#include "sys_config.h"
#include "stdlib.h"
// #include "math.h"
#include "pc_rx_data.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "data_packet.h"
#include "arm_math.h"
#include "power_control.h"
#include "imu_task.h"
#include "bsp_iwdg.h"

UBaseType_t chassis_stack_surplus;

uint16_t Start_Angle[2] = {59,96};//狗腿水平角度,车右0左1
uint16_t UP_Leg_Angle[2] = {88,66};//狗腿抬腿角度

uint8_t Up_Leg_Flag = 0;//伸腿标志位 1伸腿 0收腿
uint8_t rc_iw_ClearFlag;//防止拨轮标志位长时间置1，反复伸缩腿

//uint8_t joint_Circle[2] = {0};//狗腿圈数,车头朝自己左1右0

//uint16_t joint_ang_fdb_before[2];//上一时刻狗腿角度
uint16_t temporary_joint_ang_ref[2];//临时狗腿角度，用于过零
//uint8_t Zero_Flag[2];//狗腿过零标志位，置1顺时针过零，置-1逆时针过零

extern TaskHandle_t can_msg_send_Task_Handle;

extern pid_t pid_chassis_power_buffer; // 缓冲焦耳
// extern uint8_t twist_volt;//小陀螺标志位
extern pid_t pid_chassis_vw; // 小陀螺pid
extern pid_t pid_joint_ang_r; // 狗腿pid
extern pid_t pid_joint_ang_l; // 狗腿pid
extern pid_t pid_joint_spd_r; // 狗腿pid
extern pid_t pid_joint_spd_l; // 狗腿pid

float chassis_pitch_fdb = 0;
float chassis_pitch_ref = 0;
// 底盘控制结构体初始化
chassis_wheel_t chassis = {
	.cap_store = 27.0f,  // 电容存储容量(J)
	.fast_flag = 0,     // 快速模式标志
	.Speed_up = 0      	// 加速模式标志
};
chassis_joint_t chassis_joint[2];
float target_values[7] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};

// PID参数配置
float power_new_pid[3] = {0.0f, 0.3f, 0.0f};        // 新功率算法PID
float chassis_power_buffer_pid[3] = {1, 0, 0};     	// 缓冲能量控制PID
float chassis_vw_pid[3] = {0.0f, 0.5f, 0.0f};      	// 小陀螺转速控制PID
float chassis_yaw_cali_pid[3] = {0.0f, 0.0f, 0.0f}; // 底盘跟随YAWPID
float chassis_joint_pid[2][6] = {{40.0f, 0.12f, 0.0f,   4.0f, 0.0f, 0.01f},{40.0f, 0.12f, 0.0f,   4.0f, 0.0f, 0.01f}}; // 狗腿PID

// 四轮电机速度环PID参数
float chassis_spd_pid[4][3] = {
	{4.5, 0.1f, 0},  // 左前轮
	{4.5, 0.1f, 0},  // 右前轮
	{4.5, 0.1f, 0},  // 左后轮
	{4.5, 0.1f, 0}   // 右后轮
};

void chassis_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;
	
	chassis_joint[joint_RIGHT].joint_ang_ref = Start_Angle[joint_RIGHT];
	chassis_joint[joint_LEFT].joint_ang_ref = Start_Angle[joint_LEFT];
	
	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)INFO_GET_CHASSIS_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{

			if (Signal & INFO_GET_CHASSIS_SIGNAL)
			{
				/*底盘vw旋转的pid*/
				PID_Struct_Init(&pid_chassis_cali_angle, chassis_yaw_cali_pid[0], chassis_yaw_cali_pid[1], chassis_yaw_cali_pid[2], MAX_CHASSIS_VR_SPEED, 10, DONE);
				PID_Struct_Init(&pid_chassis_power_buffer, chassis_power_buffer_pid[0], chassis_power_buffer_pid[1], chassis_power_buffer_pid[2], 50, 10, DONE);
				PID_Struct_Init(&pid_chassis_vw, chassis_vw_pid[0], chassis_vw_pid[1], chassis_vw_pid[2], 570, 570, DONE);
				
				PID_Struct_Init(&pid_joint_spd_r, chassis_joint_pid[joint_RIGHT][0], chassis_joint_pid[joint_RIGHT][1], chassis_joint_pid[joint_RIGHT][2], 5000, 300, DONE);
				PID_Struct_Init(&pid_joint_spd_l, chassis_joint_pid[joint_LEFT][0], chassis_joint_pid[joint_LEFT][1], chassis_joint_pid[joint_LEFT][2], 5000, 300, DONE);
				PID_Struct_Init(&pid_joint_ang_r, chassis_joint_pid[joint_RIGHT][3], chassis_joint_pid[joint_RIGHT][4], chassis_joint_pid[joint_RIGHT][5], 5000, 10, DONE);
				PID_Struct_Init(&pid_joint_ang_l, chassis_joint_pid[joint_LEFT][3], chassis_joint_pid[joint_LEFT][4], chassis_joint_pid[joint_LEFT][5], 5000, 10, DONE);
				//chassis_pitch_fdb = Get_imu_Pitch();
				/*底盘vx,vy平移的pid*/
				for (int i = 0; i < 4; i++)
					PID_Struct_Init(&pid_chassis_vx_vy_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 16000, 1500, DONE);

				if (chassis_mode != CHASSIS_RELEASE) // 云台归中之后底盘才能动
				{
					switch (chassis_mode)
					{
					/*云台底盘跟随模式*/
					case CHASSIS_NORMAL_MODE:
					{
						if(!gimbal_turn_ready)
							chassis_normal_handler();
						else//掉头时底盘停止
							chassis_stop_handler();
					}
					break;
					/*小陀螺模式*/
					case CHASSIS_DODGE_MODE:
					{
						chassis_dodge_handler();
					}
					break;
					/*底盘停止模式*/
					case CHASSIS_STOP_MODE:
					{
						chassis_stop_handler();
					}
					break;

					case CHASSIS_SEPARATE_MODE:
					{
						chassis_stop_handler();
					}
			
					break;

					default:
					{
					}
					break;
					}

					// 麦克纳姆轮速度映射结算至底盘四个轮子
					mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
					//mecanum_calc(100, 0, 0, chassis.wheel_spd_ref);
					for (int i = 0; i < 4; i++)
						chassis.current[i] = pid_calc(&pid_chassis_vx_vy_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
						//计算Pitch方向和
						
					/*过零处理*/
					if(chassis_joint[joint_LEFT].joint_ang_ref - chassis_joint[joint_LEFT].joint_ang_fdb > 180)
					{
						temporary_joint_ang_ref[joint_LEFT] = chassis_joint[joint_LEFT].joint_ang_ref - 360;
					}
					else if(chassis_joint[joint_LEFT].joint_ang_ref - chassis_joint[joint_LEFT].joint_ang_fdb < -180)
					{
						temporary_joint_ang_ref[joint_LEFT] = chassis_joint[joint_LEFT].joint_ang_ref + 360;
					}
					else
					{
						temporary_joint_ang_ref[joint_LEFT] = chassis_joint[joint_LEFT].joint_ang_ref;
					}
					
					if(chassis_joint[joint_RIGHT].joint_ang_ref - chassis_joint[joint_RIGHT].joint_ang_fdb > 180)
					{
						temporary_joint_ang_ref[joint_RIGHT] = chassis_joint[joint_RIGHT].joint_ang_ref - 360;
					}
					else if(chassis_joint[joint_RIGHT].joint_ang_ref - chassis_joint[joint_RIGHT].joint_ang_fdb < -180)
					{
						temporary_joint_ang_ref[joint_RIGHT] = chassis_joint[joint_RIGHT].joint_ang_ref + 360;
					}
					else
					{
						temporary_joint_ang_ref[joint_RIGHT] = chassis_joint[joint_RIGHT].joint_ang_ref;
					}
					
						chassis_joint[joint_LEFT]. joint_spd_ref = pid_calc(&pid_joint_spd_l,chassis_joint[joint_LEFT].joint_ang_fdb,temporary_joint_ang_ref[joint_LEFT]);
						chassis_joint[joint_RIGHT].joint_spd_ref = pid_calc(&pid_joint_spd_r,chassis_joint[joint_RIGHT].joint_ang_fdb,temporary_joint_ang_ref[joint_RIGHT]);
						chassis_joint[joint_LEFT].current = pid_calc(&pid_joint_ang_l,chassis_joint[joint_LEFT].joint_spd_fdb,chassis_joint[joint_LEFT].joint_spd_ref);//继续写速度环解算
						chassis_joint[joint_RIGHT].current = pid_calc(&pid_joint_ang_r,chassis_joint[joint_RIGHT].joint_spd_fdb,chassis_joint[joint_RIGHT].joint_spd_ref);
						chassis.ob_total_power = Chassis_Power_Control(&chassis); // 功率控制

					if (!chassis_is_controllable())
					{
						memset(chassis.current, 0, sizeof(chassis.current));
						memset(&chassis_joint[joint_LEFT].current, 0, sizeof(chassis_joint[joint_LEFT].current));
						memset(&chassis_joint[joint_RIGHT].current, 0, sizeof(chassis_joint[joint_RIGHT].current));
					}
					  memcpy(glb_cur.chassis_cur, chassis.current, sizeof(chassis.current));
					  memcpy(&glb_cur.chassis_joint_cur[joint_LEFT], &chassis_joint[joint_LEFT].current, sizeof(chassis_joint[joint_LEFT].current));
					  memcpy(&glb_cur.chassis_joint_cur[joint_RIGHT], &chassis_joint[joint_RIGHT].current, sizeof(chassis_joint[joint_RIGHT].current));
				  }
				else
					memset(glb_cur.chassis_cur, 0, sizeof(glb_cur.chassis_cur));
				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)CHASSIS_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
			}
		}

		chassis_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

// extern uint8_t Speed_up;

// 新功率算法
/**
 * @brief 功率控制
 * @param power_pid  &pid_power
 * @param power_vx  &dodge_chassis_vx
 * @param power_vy &dodge_chassis_vy
 * @param real_time_Cap_remain chassis.CapData[1]
 * @param real_time_Cap_can_store cap_store
 * @param judge_power_limit (float)judge_recv_mesg.game_robot_state.chassis_power_limit
 */
void chassis_power_contorl(pid_t *power_pid, float *power_vx, float *power_vy, float real_time_Cap_remain, float real_time_Cap_can_store, float judge_power_limit)
{
	static float max = 3300;
	static float min = 1000;						  // 1000;
	static float Cap_low = CAP_LOW;					  // 何时停止加速
	float Ref_temp = 0;								  // 功率缓启动临时变量
	float cap_flag = real_time_Cap_can_store - 18.0f; // 18~real_time_Cap_can_store这一段进行按比例赋值

	if (real_time_Cap_remain < Cap_low)
		chassis.Speed_up = 1; // 一旦检测到电容低则自动关闭加速

	if (!SLOW_SPD)
	{
		Charge_factor = 1.0f;
	}
	else // 降速给底盘充电
	{
		Charge_factor = 0.6f; // 1-0.6=40% 的电量给电容充电
	}

	if (km.vy != 0 || rm.vy != 0 || km.vx != 0 || rm.vx != 0) // 底盘输入不为0,才进行pid运算（防止底盘静止时，pid->out积满）
	{
		if (chassis.Speed_up == 1) // 不加速
		{
			if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 10 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
			{
				if ((judge_recv_mesg.game_robot_state.chassis_power_limit) * Charge_factor - chassis.ob_total_power > Deta_Power) // 功率相差过大
				{
					Ref_temp = chassis.ob_total_power + Deta_Power;
					pid_calc(power_pid, chassis.ob_total_power, Ref_temp);
				}
				else
					pid_calc(power_pid, chassis.ob_total_power, (judge_recv_mesg.game_robot_state.chassis_power_limit) * Charge_factor);
			}
			else																				// 没接裁判系统
				pid_calc(power_pid, chassis.ob_total_power, (Debug_Power + 5) * Charge_factor); // 没有连接裁判系统，小车底盘期望功率达到45w
		}
	}
	if (chassis.Speed_up == 1)
	{
		if (km.vy > 0 || rm.vy > 0)
		{
			*power_vy = (min + power_pid->out);
			VAL_LIMIT(*power_vy, min * 0.7f, max);
		}
		else if (km.vy < 0 || rm.vy < 0)
		{
			*power_vy = -(min + power_pid->out);
			VAL_LIMIT(*power_vy, -max, -min * 0.7f);
		}
		else
			*power_vy = 0;

		if (km.vx > 0 || rm.vx > 0)
		{
			*power_vx = (min + power_pid->out);
			VAL_LIMIT(*power_vx, min, max);
		}
		else if (km.vx < 0 || rm.vx < 0)
		{
			*power_vx = -(min + power_pid->out);
			VAL_LIMIT(*power_vx, -max, -min);
		}
		else
			*power_vx = 0;
	}
	else if (chassis.Speed_up == 0)
	{
		if (km.vy > 0 || rm.vy > 0)
		{
			*power_vy = CHASSIS_KB_MAX_SPEED_Y;
		}
		else if (km.vy < 0 || rm.vy < 0)
		{
			*power_vy = -CHASSIS_KB_MAX_SPEED_Y;
		}
		else
			*power_vy = 0;

		if (km.vx > 0 || rm.vx > 0)
		{
			*power_vx = CHASSIS_KB_MAX_SPEED_X;
		}
		else if (km.vx < 0 || rm.vx < 0)
		{
			*power_vx = -CHASSIS_KB_MAX_SPEED_X;
		}
		else
			*power_vx = 0;
	}

	if (chassis.fast_flag && (rc.ch2 == 660 || FAST_SPD)) // 快速
	{
		chassis.Speed_up = 0;
	}
	else // 正常速度
	{
		chassis.Speed_up = 1;
	}

	if (real_time_Cap_remain > Cap_low && !FAST_SPD)
	{
		chassis.fast_flag = 1;
	} // 防止加速过程中反复开关加速模式（抽搐）
	if (real_time_Cap_remain < Cap_low)
	{
		chassis.fast_flag = 0;
	}
}

// 普通模式
static void chassis_normal_handler(void)
{

	float normal_chassis_vx, normal_chassis_vy; // 临时vx、vy速度变量
	// 计算底盘x、y轴上的速度
	chassis_power_contorl(&pid_power, &normal_chassis_vx, &normal_chassis_vy, chassis.CapData[1], chassis.cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	if (km.face)																																			  // 横移
		chassis.vw = (-pid_calc(&pid_chassis_cali_angle, gimbal.sensor.yaw_relative_angle, gimbal.sensor.yaw_total_angle - gimbal.yaw_offset_angle + 90.0f)); // + chassis_offset_angle));//底盘禁止时底盘锁住最后一刻陀螺仪方向。
	else
		chassis.vw = (-pid_calc(&pid_chassis_cali_angle, gimbal.sensor.yaw_relative_angle, gimbal.sensor.yaw_total_angle - gimbal.yaw_offset_angle)); // + chassis_offset_angle));//底盘禁止时底盘锁住最后一刻陀螺仪方向。

	//由二维旋转公式得到x轴与y轴的值
	//chassis.vy = (normal_chassis_vx * arm_sin_f32(PI / 180 * gimbal.sensor.yaw_relative_angle) + normal_chassis_vy * arm_cos_f32(PI / 180 * gimbal.sensor.yaw_relative_angle));
	//chassis.vx = (normal_chassis_vx * arm_cos_f32(PI / 180 * gimbal.sensor.yaw_relative_angle) - normal_chassis_vy * arm_sin_f32(PI / 180 * gimbal.sensor.yaw_relative_angle));
	
	chassis.vy = normal_chassis_vx * CHASSIS_RC_MOVE_RATIO_X ;
	chassis.vx = normal_chassis_vy * CHASSIS_RC_MOVE_RATIO_Y ;
	chassis.vw = rm.vw * CHASSIS_RC_MOVE_RATIO_R;
	
	/*抬腿*/
	if(rc.iw_flag == 1 && rc_iw_ClearFlag == 1)
	{
		if(Up_Leg_Flag == 0)
		{
			chassis_joint[joint_RIGHT].joint_ang_ref = UP_Leg_Angle[joint_RIGHT];
			chassis_joint[joint_LEFT].joint_ang_ref = UP_Leg_Angle[joint_LEFT];
			
//			if(chassis_joint[joint_RIGHT].joint_ang_ref < 0)
//			{
//				joint_Circle[joint_RIGHT]++;
//			}
//			if(chassis_joint[joint_LEFT].joint_ang_ref > 360)
//			{
//				joint_Circle[joint_LEFT]--;
//			}
			
//			chassis_joint[joint_RIGHT].joint_ang_ref += 360 * joint_Circle[joint_RIGHT];
			
			Up_Leg_Flag = 1;
		}
		else
		{
			chassis_joint[joint_RIGHT].joint_ang_ref = Start_Angle[joint_RIGHT];
			chassis_joint[joint_LEFT].joint_ang_ref = Start_Angle[joint_LEFT];
			
			Up_Leg_Flag = 0;
		}
		rc_iw_ClearFlag = 0;
	}
	else if(rc.iw_flag == 0)
	{
		rc_iw_ClearFlag = 1;
	}
}

// 小陀螺模式
#define dodge_min -500
#define dodge_max 500
static void chassis_dodge_handler(void)
{
	float dodge_angle;
	float dodge_chassis_vx, dodge_chassis_vy;

	if (last_chassis_mode != CHASSIS_DODGE_MODE)
	{
		PID_Clear(&pid_chassis_vw);
	}

	/*小陀螺*/
	dodge_angle = gimbal.sensor.yaw_relative_angle;
	//
	if ((rm.vx != 0 || rm.vy != 0) && (km.vx == 0 || km.vy == 0)) // 小陀螺时遥控方向降低速度
	{
		dodge_chassis_vx = rm.vx * 0.5f;
		dodge_chassis_vy = rm.vy * 0.5f;
	}
	else
		// 键盘时则通过功率算法来控制速度，防止掉电容
		chassis_power_contorl(&pid_power, &dodge_chassis_vx, &dodge_chassis_vy, chassis.CapData[1], chassis.cap_store, (float)judge_recv_mesg.game_robot_state.chassis_power_limit);

	chassis.vy = (dodge_chassis_vx * arm_sin_f32(PI / 180 * dodge_angle) + dodge_chassis_vy * arm_cos_f32(PI / 180 * dodge_angle));
	chassis.vx = (dodge_chassis_vx * arm_cos_f32(PI / 180 * dodge_angle) - dodge_chassis_vy * arm_sin_f32(PI / 180 * dodge_angle));

	if (chassis.Speed_up == 1) // 不加速
	{
		if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 10 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
		{
			pid_calc(&pid_chassis_vw, chassis.ob_total_power, (judge_recv_mesg.game_robot_state.chassis_power_limit) * Charge_factor);
		}
		else																					  // 没接裁判系统
			pid_calc(&pid_chassis_vw, chassis.ob_total_power, (Debug_Power + 5) * Charge_factor); // 没有连接裁判系统，小车底盘期望功率达到45w
	}
	else if (chassis.Speed_up == 0) // 按下加速功率多100w
	{
		if (judge_recv_mesg.game_robot_state.chassis_power_limit >= 10 && judge_recv_mesg.game_robot_state.chassis_power_limit <= 220)
		{
			pid_calc(&pid_chassis_vw, chassis.ob_total_power, (judge_recv_mesg.game_robot_state.chassis_power_limit + 105));
		}
		else // 没接裁判系统
		{
			pid_calc(&pid_chassis_vw, chassis.ob_total_power, 105 + Debug_Power); // 没有连接裁判系统，小车底盘期望功率达到150w
		}
	}

	if (rc.iw <= IW_UP)
	{
		chassis.vw = -(pid_chassis_vw.out);
		if (chassis.vw < (dodge_min))
			chassis.vw = (dodge_min);
		if (chassis.vw > (dodge_max))
			chassis.vw = (dodge_max);
	}
	else
	{
		chassis.vw = pid_chassis_vw.out;
		if (chassis.vw < dodge_min)
			chassis.vw = dodge_min;
		if (chassis.vw > dodge_max)
			chassis.vw = dodge_max;
	}
	
}

// 停止模式
static void chassis_stop_handler(void)
{
	chassis.vy = 0;
	chassis.vx = 0;
	chassis.vw = 0;
}
// 初始化
void chassis_param_init(void)
{
	memset(&chassis, 0, sizeof(chassis));

	/*底盘vw旋转的pid*/
	PID_Struct_Init(&pid_chassis_cali_angle, chassis_yaw_cali_pid[0], chassis_yaw_cali_pid[1], chassis_yaw_cali_pid[2], MAX_CHASSIS_VR_SPEED, 50, INIT);
	PID_Struct_Init(&pid_power, power_new_pid[0], power_new_pid[1], power_new_pid[2], 3000, 2000, INIT); // 新功率算法

	/*底盘vx,vy平移的pid*/
	for (int i = 0; i < 4; i++)
		PID_Struct_Init(&pid_chassis_vx_vy_spd[i], chassis_spd_pid[i][0], chassis_spd_pid[i][1], chassis_spd_pid[i][2], 10000, 500, INIT);
}

/**
 * @brief mecanum chassis velocity decomposition
 * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
 *        output: every wheel speed(rpm)
 * @trans 输入：		前后左右的量
 *		  输出：		每个轮子对应的速度
 * @note  1=FR 2=FL 3=BL 4=BR
 * @work	 分析演算公式计算的效率
 */

#define rotate_ratio_fr ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF
#define rotate_ratio_fl ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF
#define rotate_ratio_bl ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET) / RADIAN_COEF
#define rotate_ratio_br ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET) / RADIAN_COEF
#define wheel_rpm_ratio 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO)

static void mecanum_calc(float vy, float vx, float vw, int16_t speed[]) // 底盘解算，得到底盘获得相应速度需要的四个电机值
{
	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); // mm/s
	VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); // mm/s
	/*小陀螺以外的模式，vw限制正常*/
	if (chassis_mode != CHASSIS_DODGE_MODE && !chassis.dodge_ctrl)
		VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED); // deg/s

	int16_t wheel_rpm[4];
	float max = 0;

//	wheel_rpm[0] = (vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
//	wheel_rpm[1] = (-vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
//	wheel_rpm[2] = (-vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
//	wheel_rpm[3] = (vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

  wheel_rpm[0] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = ( vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = ( vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;
	
	// find max item
	for (uint8_t i = 0; i < 4; i++)
	{
		if (abs(wheel_rpm[i]) > max)
			max = abs(wheel_rpm[i]);
	}
	// equal proportion
	if (max > MAX_WHEEL_RPM)
	{
		float rate = MAX_WHEEL_RPM / max;
		for (uint8_t i = 0; i < 4; i++)

			wheel_rpm[i] *= rate;
	}
	memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}
