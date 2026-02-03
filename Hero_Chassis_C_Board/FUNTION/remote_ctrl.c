#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "shoot_task.h"
#include "keyboard.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"
#include "chassis_task.h"
#include "gimbal_task.h"

rc_ctrl_t rm;//遥控控制变量
sw_record_t glb_sw;//记录上一次拨杆值

/*
* @ RC_RESOLUTION :摇杆最大值 660 
* @ CHASSIS_RC_MAX_SPEED_X
    CHASSIS_RC_MAX_SPEED_Y
    CHASSIS_RC_MAX_SPEED_R ：平移和旋转的速度最大值
  @ CHASSIS_RC_MOVE_RATIO_X
    CHASSIS_RC_MOVE_RATIO_Y
    CHASSIS_RC_MOVE_RATIO_R : 数值方向
*/
static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate)
{
	rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X * CHASSIS_RC_MOVE_RATIO_X; //右↑ y
	rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y * CHASSIS_RC_MOVE_RATIO_Y;  // 右← x	  
	rm.vw = rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R * CHASSIS_RC_MOVE_RATIO_R*0.5f;
	if(chassis.direction==0)
	{
		rm.vx=-rm.vx;
		rm.vy=-rm.vy;
	}
}
static void chassis_dodge_ctrl(void)
{
	if(chassis.dodge_ctrl && rc.iw_flag != 0 && rc.iw_flag == 1)
	{
		rc.iw  = -500;
	}
	else
		rc.iw = 0;
}

void remote_ctrl_chassis_hook(void)
{
	chassis_operation_func(rc.ch2, rc.ch1,rc.ch3);
	chassis_dodge_ctrl();
}

