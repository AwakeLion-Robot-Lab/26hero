#include "comm_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "bsp_can.h"
#include "modeswitch_task.h"
UBaseType_t can_stack_surplus;

motor_current_t glb_cur;
void can_msg_send_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)GIMBAL_MOTOR_MSG_SIGNAL |
									CHASSIS_MOTOR_MSG_SIGNAL |
									SHOT_MOTOR_MSG_SIGNAL |
									MODE_SWITCH_MSG_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{

			if (!soft_init_flag)
			{
				if (Signal & GIMBAL_MOTOR_MSG_SIGNAL) // 发送云台电流
				{
					send_gimbal_yaw_cur(glb_cur.gimbal_cur[YAW]);	// yaw CAN1 ID 6
					send_gimbal_pit_cur(glb_cur.gimbal_cur[PITCH]); // pit CAN2 ID 1
				}

				if (Signal & SHOT_MOTOR_MSG_SIGNAL) // 发送摩擦轮电流
				{
					// 水平摩擦轮与底盘一致，逆时针递增RIGHT_FRIC_FRONT
					send_fric_cur(glb_cur.fric_cur[FRIC_DOWN], glb_cur.fric_cur[RIGHT_FRIC_UP], glb_cur.fric_cur[LEFT_FRIC_UP] ); 
					 
				}    
			}        
			else
			{
//				send_gimbal_yaw_cur(0);
//				send_gimbal_pit_cur(0);
				send_fric_cur(0, 0, 0);

			}
			if (Signal & MODE_SWITCH_MSG_SIGNAL) // 发送电流全为0
			{
				send_gimbal_yaw_cur(0);
				send_gimbal_pit_cur(0);
				send_fric_cur(0, 0, 0);
				if (soft_init_flag == 1)
				{
					soft_init_flag = 0;
					Stm32_SoftReset();
				}
			}
		}

		can_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}
