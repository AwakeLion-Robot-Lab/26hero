#ifndef _chassis_task_H
#define _chassis_task_H

#include "stm32f4xx.h"

typedef struct
{
  uint8_t         dodge_ctrl;      // 小陀螺控制标志
  uint8_t         separt_ctrl;     // 分离控制标志
  uint8_t         direction;       // 运动方向
  float           vx;              // 前后方向速度(m/s)
  float           vy;              // 左右方向速度(m/s)
  float           vw;              // 旋转角速度(rad/s)
  int16_t         rotate_x_offset; // X轴旋转偏移
  int16_t         rotate_y_offset; // Y轴旋转偏移
  
  int16_t         wheel_spd_fdb[4]; // 轮速反馈值
  int16_t         wheel_spd_ref[4]; // 轮速参考值
  int16_t         current[4];       // 电机电流值
  double          CapData[4];       // 电容数据
  uint8_t         cap_volt_warning; // 电容电压警告
  uint8_t         fast_flag;        // 快速模式标志
  uint8_t         Speed_up;         // 加速模式标志
  float           ob_total_power;   // 当前总功率
  float           cap_store;        // 超级电容容量(最大27)
}chassis_wheel_t;

typedef struct
{
  
  int16_t         joint_spd_fdb; // 轮速反馈值
  int16_t         joint_spd_ref; // 轮速参考值
	int16_t         joint_ang_fdb; // 角度反馈值
  int16_t         joint_ang_ref; // 角度参考值
  int16_t         current;       // 电机电流发送值
	int16_t			current_fdb;//电机电流返回值
	uint8_t         P;

}chassis_joint_t;

enum Dog_joint//车子为准
{
	 joint_RIGHT = 0,
	 joint_LEFT = 1
};

//typedef struct
//{
//  
// 
//	float Pitch_angle;
//	void (*Motor_Init)(void);
//	void (*Motor_Set_angle)(void);

//}chassis_t;

extern chassis_wheel_t chassis;
extern chassis_joint_t chassis_joint[2];
//extern uint8_t joint_Circle[2];
//extern uint16_t joint_ang_fdb_before[2];
//extern uint8_t Zero_Flag[2];
void chassis_task(void *parm);
void chassis_param_init(void);

static void chassis_normal_handler(void);
static void chassis_separate_handler(void);
static void chassis_dodge_handler(void);
static void chassis_stop_handler(void);
static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

#endif
