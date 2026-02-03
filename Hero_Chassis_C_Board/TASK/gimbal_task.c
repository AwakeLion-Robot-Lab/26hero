#include "gimbal_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "modeswitch_task.h"
#include "chassis_task.h"
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
#include "shoot_task.h"

UBaseType_t gimbal_stack_surplus;
extern TaskHandle_t can_msg_send_Task_Handle;
uint8_t gimbal_turn_flag;	//掉头标志位
uint32_t turn_time;			//掉头时间
uint8_t gimbal_turn_ready;	//掉头完成标志位
gimbal_t gimbal; 			// 云台控制结构体
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
				//判断掉头时长
				if(gimbal_turn_flag)
				{
					turn_time = HAL_GetTick();
					gimbal_turn_ready = 1;
				}
				if(!gimbal_turn_flag && (HAL_GetTick() - turn_time > 950)) //延迟950ms
				{
					gimbal_turn_ready = 0;
				}
					
				last_gimbal_mode = gimbal_mode; // 获取上一次云台状态

				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)GIMBAL_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
			}
		}

		gimbal_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

