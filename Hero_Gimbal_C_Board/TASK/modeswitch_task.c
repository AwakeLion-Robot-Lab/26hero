#include "modeswitch_task.h"
#include "STM32_TIM_BASE.h"
#include "detect_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "remote_ctrl.h"
#include "keyboard.h"
#include "comm_task.h"
#include "gimbal_task.h"
#include "bsp_can.h"
#include "math.h"
#include "arm_math.h"
#include "pc_rx_data.h"
#include "bsp_iwdg.h"  

UBaseType_t mode_switch_stack_surplus;

extern uint8_t gimbal_turn_flag;
extern TaskHandle_t info_get_Task_Handle;
extern uint8_t shoot_ready; //拨盘运动标志位

global_status global_mode; //全局模式
global_status last_global_mode;

gimbal_status gimbal_mode; //云台模式
gimbal_status last_gimbal_mode;

chassis_status chassis_mode; //底盘模式
chassis_status last_chassis_mode;

shoot_status shoot_mode; //发射模式
shoot_status last_shoot_mode;

uint8_t soft_init_flag=0;
//软件复位
void Stm32_SoftReset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

void keyboard_init_hook()
{
	if(gimbal.soft_reset_flag)
		soft_init_flag=1;
			
}
void mode_switch_task(void *parm)
{
	uint32_t mode_switch_wake_time = osKernelSysTick();
  while(1)
  {
    get_main_mode();
    get_gimbal_mode();
	  
	  
	keyboard_init_hook();//软件重启
	  
	//发送pit的相对角度和视觉有效位以及轻触开关是否触碰到的标志位
	send_to_chassis(gimbal.sensor.pit_relative_angle,pc_recv_mesg.mode_Union.info.visual_valid,shoot_ready);
	send_yaw_to_chassis(gimbal.sensor.yaw_total_angle,gimbal.yaw_offset_angle);
	gimbal_to_chassis_rc();
	/**************************************************************************/
    xTaskGenericNotify( (TaskHandle_t) info_get_Task_Handle, 
                  (uint32_t) MODE_SWITCH_INFO_SIGNAL, 
                  (eNotifyAction) eSetBits, 
                  (uint32_t *)NULL );
	IWDG_Feed();//喂狗
	  
    mode_switch_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
    
    vTaskDelayUntil(&mode_switch_wake_time, 2);
  }
}
/*获取上一次的所有模式，包括全局、云台、底盘、发射机构*/
void get_last_mode(void)
{
  last_global_mode = global_mode;//获取上一次全局状态
  last_gimbal_mode = gimbal_mode;//获取上一次云台状态
  last_chassis_mode = chassis_mode;//获取上一次底盘状态
  last_shoot_mode = shoot_mode;//获取上一次射击状态
}

static uint8_t key_state(uint8_t now_state, image_key_status image_key)
{
	static uint8_t last_l_state = 0;
	static uint8_t last_r_state = 0;
	static uint8_t last_m_state = 0;
	static uint8_t last_l_motion = 0;
	static uint8_t last_r_motion = 0;
	static uint8_t last_m_motion = 0;
	switch(image_key)
	{
		case IMAGE_L_KEY:
		if(last_l_state == 0 && now_state)
			last_l_motion = !last_l_motion;
		last_l_state = now_state;
		return last_l_motion;
		case IMAGE_R_KEY:
		if(last_r_state == 0 && now_state)
			last_r_motion = !last_r_motion;
		last_r_state = now_state;
		return last_r_motion;
		case IMAGE_M_KEY:
		if(last_m_state == 0 && now_state)
			last_m_motion = !last_m_motion;
		last_m_state = now_state;
		return last_m_motion;
		default:
		last_l_motion = 0;
		last_r_motion = 0;
		last_m_motion = 0;
		return 0;
	}
}
void get_main_mode(void)
{   
	if (global_err.list[REMOTE_CTRL_OFFLINE].err_exist == 1) // 遥控离线或YAW电机掉电
	{	
		global_mode = RELEASE_CTRL; //!
	}
	else if((rc.sw2 == RC_MI) || (rc.image.sw == RC_N))
	{

		if(TRACK_CTRL || key_state(rc.image.key_l, IMAGE_L_KEY))//鼠标右键进入视觉
			global_mode = SEMI_AUTOMATIC_CTRL;
		else
			global_mode = MANUAL_CTRL;
	}
	else if(rc.sw2 == RC_UP || rc.image.sw == RC_S)
	{
			global_mode = SEMI_AUTOMATIC_CTRL;
	}
	else if(rc.sw2 == RC_DN || rc.image.sw == RC_C)//①机器人阵亡、pitch/yaw电机离线、遥控离线；②遥控右拨杆现实下打
	{
		global_mode = RELEASE_CTRL;//引发云台预备归中
	}
	else//①机器人阵亡、pitch/yaw电机离线、遥控离线；②遥控右拨杆现实下打
	{
		global_mode = RELEASE_CTRL;//引发云台预备归中
	}
}
static uint8_t kb_dodge_enabled = 0; //键盘启动标志位
/*根据`global_mode`进行云台模式选择*/
void get_gimbal_mode(void)
{  		
	/* 小陀螺模式标志位解码 */
	if (KB_DODGE_CTRL )
		kb_dodge_enabled = 1;
	if (KB_DODGE_CTRL_CLOSE)
		kb_dodge_enabled = 0; 
	if(kb_dodge_enabled || RC_DODGE_MODE)
	{
		gimbal.dodge_ctrl = ON;		//小陀螺模式开关置1
		gimbal.separt_ctrl = OFF; 	// 分离模式开关置0
	}
	else
	{
		gimbal.dodge_ctrl = OFF;	
	}
  
  /* 分离模式标志位解码 */
  if (KB_SEPART_OPEN) 
  {
    gimbal.separt_ctrl = ON; // 分离模式开关置1
    gimbal.dodge_ctrl = OFF; // 小陀螺模式开关置0
  }
  if (KB_SEPART_CLOSE || KB_GIMBAL_TURN_BACK)
  {
    gimbal.separt_ctrl = OFF;  // 分离模式开关置0
  }

  switch(global_mode)
  {
    case MANUAL_CTRL://正常
    {
			if(gimbal.state == GIMBAL_INIT_NEVER)
			{
				 gimbal_mode = GIMBAL_INIT;//云台归中
			}
			else //归中完毕后才执行配合底盘的操作
			{
				if(gimbal.dodge_ctrl)
					gimbal_mode = GIMBAL_DODGE_MODE; //小陀螺模式
				else if(gimbal.separt_ctrl)
					gimbal_mode = GIMBAL_SEPARATE_MODE; //分离模式
				else
					gimbal_mode = GIMBAL_NORMAL_MODE; //云台底盘跟随模式
			}
    }
    break;
    
    case SEMI_AUTOMATIC_CTRL: //自瞄
    {
			if(gimbal.state == GIMBAL_INIT_NEVER)//当底盘传上来的 gimbal.state 标志位 为 GIMBAL_INIT_NEVER 时，云台首先进行归中
				 gimbal_mode = GIMBAL_INIT;//云台归中
			else //归中完毕后才执行配合底盘的操作{
				gimbal_mode = GIMBAL_TRACK_ARMOR; //云台自瞄模式}
    }
    break;
    
    case RELEASE_CTRL:
    {
      gimbal_mode = GIMBAL_RELEASE;
      gimbal.state = GIMBAL_INIT_NEVER;
    }break;
    
  }
}
