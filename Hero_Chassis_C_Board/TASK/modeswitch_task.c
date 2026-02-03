#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"
#include "judge_task.h"
#include "judge_rx_data.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "detect_task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "pid.h"
#include "bsp_can.h"
#include "shoot_task.h"
#include "bsp_iwdg.h"
UBaseType_t mode_switch_stack_surplus;

extern TaskHandle_t info_get_Task_Handle;
uint8_t doge_twist;                   // 小陀螺低电压标志位

// 全局拨杆
global_status global_mode;      // 当前拨杆状态
global_status last_global_mode; // 上一次拨杆状态
// 云台
gimbal_status gimbal_mode;      // 当前云台模式
gimbal_status last_gimbal_mode; // 上一次云台模式
// 底盘
chassis_status chassis_mode;      // 当前底盘模式
chassis_status last_chassis_mode; // 上一次底盘模式

char switch_status[2] = {0}; // 发送模式给云台开发板

void mode_switch_task(void *parm)
{
  uint32_t mode_switch_wake_time = osKernelSysTick();
  while (1)
  {
    get_last_mode();    // 获取上一次模式
    get_chassis_mode(); // 根据正常模式/自瞄模式与相关标志位判断底盘该处于小陀螺还是分离还是跟随(该部分重要的是标志位的判断【KB、RC、Capacity】)

    switch_status[LEFT_SWITCH] = rc.sw1;
    switch_status[RIGHT_SWITCH] = rc.sw2;

    // 遥控器未开启时，把各个标志位置0；机器人死亡时也置0（此处以yaw轴电机是否得电判断机器人是否死亡，由于开发板单独供电，故需这么做）
    if (global_err.list[REMOTE_CTRL_OFFLINE].err_exist == 1 || global_err.list[GIMBAL_YAW_OFFLINE].err_exist == 1)
    {
      switch_status[RIGHT_SWITCH] = RC_DN; // 软件下打
      chassis.dodge_ctrl = 0;              // 小陀螺失能
      chassis.separt_ctrl = 0;             // 分离模式失能
      km.face = 0;                         // 侧身模式失能
    }
    send_judge_to_gimbal(); // 发送裁判系统值给云台开发板                                                                         // cv按键，X倍镜开关按键
    IWDG_Feed();            // 喂狗
    /****************************TODO:在有电机离线时，可通过开发板上的灯判断那个电机离线***************************************/

    /*******************************************************************/

    xTaskGenericNotify((TaskHandle_t)info_get_Task_Handle,
                       (uint32_t)MODE_SWITCH_INFO_SIGNAL,
                       (eNotifyAction)eSetBits,
                       (uint32_t *)NULL);

    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);

    vTaskDelayUntil(&mode_switch_wake_time, 2);
  }
}

void get_last_mode(void)
{
  last_global_mode = global_mode;   // 获取上一次全局状态
  last_gimbal_mode = gimbal_mode;   // 获取上一次云台状态
  last_chassis_mode = chassis_mode; // 获取上一次底盘状态
}
/*********************原理是利用遥控器拨杆传递数据，从而进行机器人的模式切换*********************/

// 底盘
void get_chassis_mode(void)
{

  if(gimbal.dodge_ctrl == ON)
  {
    chassis.dodge_ctrl = ON;   // 小陀螺开关置1
    chassis.separt_ctrl = OFF; // 分离模式开关置0
  }

  if(gimbal.dodge_ctrl == OFF) // 若KB小陀螺关闭，且电压不足，小陀螺开关置0
  {
    chassis.dodge_ctrl = OFF; // 小陀螺模式开关置0
  
  }
  /* 分离模式标志位解码 */
  if(gimbal.separt_ctrl == ON) 
  {
    chassis.separt_ctrl = ON; // 分离模式开关置1
    chassis.dodge_ctrl = OFF; // 小陀螺模式开关置0

  }
  if(gimbal.separt_ctrl == OFF)
  {
    chassis.separt_ctrl = OFF;  // 分离模式开关置0
  }

  switch(global_mode)
  {

	case MANUAL_CTRL: // 正常
	{
		if (chassis.dodge_ctrl == ON) // 如果开启躲避模式，底盘进入小陀螺
			chassis_mode = CHASSIS_DODGE_MODE;       // 小陀螺模式
		else if (chassis.separt_ctrl == ON)
			chassis_mode = CHASSIS_SEPARATE_MODE; // 分离模式
		else
			chassis_mode = CHASSIS_NORMAL_MODE; // 底盘云台正常模式(跟随)
		
	}
	break;

	case SEMI_AUTOMATIC_CTRL: // 自瞄
	{
		if (chassis.dodge_ctrl == ON) // 如果开启躲避模式，底盘进入小陀螺
			chassis_mode = CHASSIS_DODGE_MODE;       // 小陀螺模式
		else if (chassis.separt_ctrl == ON)
			chassis_mode = CHASSIS_SEPARATE_MODE; // 分离模式
		else
			chassis_mode = CHASSIS_NORMAL_MODE; // 底盘云台正常模式(跟随)
	}
	break;

	case RELEASE_CTRL: // 关闭遥控器
	{
		chassis_mode = CHASSIS_RELEASE;
		for (int i = 0; i < 4; i++)
			glb_cur.chassis_cur[i] = pid_calc(&pid_chassis_vx_vy_spd[i], chassis.wheel_spd_fdb[i], 0); // 机器人底盘无力
		km.face = 0;                                                                                 // 取消侧身
		chassis.dodge_ctrl = OFF; 
		chassis.separt_ctrl = OFF;  // 分离模式开关置0
	  // 取消小陀螺
	}
	break;

	default:
	{
	}
	break;
	
  }
}
