#ifndef _gimbal_task_H
#define _gimbal_task_H


#include "stm32f4xx.h"
#include "ramp.h"

typedef enum // 云台跟踪状态枚举
{
  REF_KEEP,    // 保持当前状态
  REF_CONFIRM, // 确认目标
  REF_ADD,     // 增加跟踪
  REF_SUB,     // 减少跟踪
} trace_fdb_t;

typedef struct
{
  /* 角度环的给定和反馈 */
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* 速度环的给定和反馈 */
  float yaw_spd_ref;
  float pit_spd_ref;
  float yaw_spd_fdb;
  float pit_spd_fdb;
  /* 记录分离模式下的目标值 */
  float seprarte_yaw_ref; 
  float seprarte_pit_ref;
} gim_pid_t;

typedef struct
{
  /* 相对角度，陀螺仪欧拉角 */
  float pit_relative_angle;
  float yaw_relative_angle;
  float roll_relative_angle;
  float pit_gyro_angle;
  float yaw_gyro_angle;
  float roll_gyro_angle;
  /* 角速度 */
  float yaw_palstance;
  float pit_palstance;
  float roll_palstance;
	
  float yaw_total_angle;
  int32_t yaw_cnt;
	
} gim_sensor_t;


typedef struct
{
	uint8_t track_ctrl;
	uint8_t dodge_ctrl;
	uint8_t separt_ctrl; // 分离控制标志
  
  gim_pid_t pid;       // PID控制结构
  gim_sensor_t sensor; // 传感器数据
	/*目标跟踪*/
	trace_fdb_t 	trace_state;
	/*从FLASH中读出的校准编码位*/
	int32_t       pit_center_offset;
	int32_t       yaw_center_offset;
	float         yaw_offset_angle;
	float 	 	 	  yaw_angle;
	uint8_t			  soft_reset_flag;
}gimbal_t;


/*前馈控制参数*/
typedef struct
{
	//前馈补偿参数
	float a;
	float b;
	//缓存参数
	float rin;
	float lastRin;
	float perrRin;
}FFC;

void gimbal_task(void *parm);
extern gimbal_t gimbal;
extern uint8_t input_flag;
extern uint8_t gimbal_turn_flag;
extern uint8_t gimbal_turn_ready;

#endif

