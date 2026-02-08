#include "shoot_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rc.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "string.h"
#include "pc_rx_data.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include "chassis_task.h"

UBaseType_t shoot_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;

extern uint8_t shoot_ready;
uint8_t last_shoot_ready;

uint8_t fire_once;					//执行一次标志位
uint8_t loader_angle_flag = 0;

shoot_t shoot;
loader_t loader;

// 拨盘电机PID控制参数
static const float loader_pid[6] = {45, 0, 0, 20, 0, 0};

// 射击系统状态变量
float current_heat = 0;   		// 当前热量值
float shoot_already_flag = 0;  	// 射击完成标志位

void shoot_task(void *parm)
{
	uint32_t signal;
	BaseType_t status;

	while (1)
	{
		status = xTaskNotifyWait((uint32_t)NULL,
							(uint32_t)INFO_GET_SHOOT_SIGNAL,
							(uint32_t *)&signal,
							portMAX_DELAY);
		if (status == pdTRUE && (signal & INFO_GET_SHOOT_SIGNAL))
		{
				// 初始化拨盘电机PID控制器
				PID_Struct_Init(&pid_loader_angle, loader_pid[0], loader_pid[1], loader_pid[2], 8000, 5000, DONE);
				PID_Struct_Init(&pid_loader_spd, loader_pid[3], loader_pid[4], loader_pid[5], 16000, 8000, DONE);
				
				if (shoot.fric_trun_on) // 如果水平摩擦轮是打开的
				{
					if(!shoot.loader_fire_allow)fire_once = 1;

					shoot_bullet_handler(); // 判断shoot_ready控制拨盘供弹
					
					if(loader_angle_flag)
					{
						pid_calc(&pid_loader_angle, moto_loader.total_angle, loader.angle_ref);
						loader.spd_ref = pid_loader_angle.out;
					} 
					pid_calc(&pid_loader_spd, moto_loader.speed_rpm, loader.spd_ref);
				}
				else
				{
					loader.angle_ref = 0;
					loader.spd_ref = 0;
					pid_loader_spd.out = 0;
					pid_loader_spd.iout = 0;
					pid_loader_angle.out = 0;
					pid_loader_angle.iout = 0;
					loader.angle_ref = moto_loader.total_angle; // 记录当前拨盘电机编码位
				}
				last_shoot_ready = shoot_ready;
				
				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)SHOT_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
		}
		shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}
uint32_t delay_loader_time = 0;
static void shoot_bullet_handler(void)
{
	if(!last_shoot_ready && shoot_ready )
	{
		loader.angle_ref = moto_loader.total_angle;
	}
	if(shoot_ready && shoot.loader_fire_allow && fire_once)
	{	
		loader_angle_flag = 1;
		loader.angle_ref -= 72 ;
		fire_once = 0;
		delay_loader_time = HAL_GetTick();
	}
	else if (shoot_ready == 0 && HAL_GetTick() - delay_loader_time > 500)
	{
		loader_angle_flag = 0;
		loader.spd_ref = -500;
	}
	else if (shoot_ready == 1 && last_shoot_ready == 0)
	{
		loader_angle_flag = 0;
		loader.spd_ref = 0;
		delay_loader_time = HAL_GetTick();
	}
}

void shoot_param_init(void)
{
	memset(&shoot, 0, sizeof(shoot_t));
	memset(&loader, 0, sizeof(loader_t));

	PID_Struct_Init(&pid_loader_angle, loader_pid[0], loader_pid[1], loader_pid[2], 8000, 300, DONE);
	PID_Struct_Init(&pid_loader_spd, loader_pid[3], loader_pid[4], loader_pid[5], 16000, 10000, DONE);
}



